# _partI.py
# Task i, use the mazzer tool to turn the wdt rotor five full revolutions
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import math
import tools
from _common import get_point, build_pose, rotate_about_tip
from _partGH import compute_wdt_pose


def compute_task_i_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm=0):
    wdt_pose = compute_wdt_pose(fixture_fine_tune_mm)

    # Sets the placement point as the rotor fastener hole (point 46), with the tool pointing straight down
    # into the rotor along the wdt's negative z axis, the rotation about that axis found by inspection
    point_local = get_point(46)
    axis_mapping = [
        [1, 0, 0],   # Tool x axis lines up with the fixture's x axis
        [0, -1, 0],  # Tool y axis lines up with the fixture's negative y axis
        [0, 0, -1],  # Tool z axis lines up with the fixture's negative z axis, pointing down into the rotor
    ]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    # Lifts the target straight up along the wdt's z by hover_mm, so the tool can line up above the hole first
    T_world_target = wdt_pose * transl(0, 0, hover_mm) * T_fixture_placement

    # Sets the mazzer tool's contact point, the cushioning cylinder tip that goes into the rotor hole
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    task_i_pose = T_world_target * T_tool_offset.inv()

    # Applies the approach angle correction, rotated about the tool tip so its position doesn't move
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_i_pose = rotate_about_tip(task_i_pose, tool_point_local, angle_adjust)

    return task_i_pose


def compute_task_i_orbit_steps(fixture_fine_tune_mm, approach_angle_adjust_deg, pivot_fine_tune_mm,
                               num_revolutions, steps_per_revolution, spin_direction):
    # Moves the tool tip around a circle centred on the rotor's axis, and the tool keeps the SAME orientation the
    # whole way round. The rotor turns because the tip sits in an off centre hole, like a crank. Keeping the
    # orientation fixed matters because turning the tool itself five times over would wind the wrist joint past
    # its limit, but only the position needs to travel round the circle
    wdt_pose = compute_wdt_pose(fixture_fine_tune_mm)
    engaged_pose = compute_task_i_pose(fixture_fine_tune_mm, approach_angle_adjust_deg)

    # The rotor axis runs along the wdt's z through the wdt frame origin, shifted by pivot_fine_tune_mm
    pivot_pose = wdt_pose * transl(*pivot_fine_tune_mm)
    pivot_x, pivot_y = pivot_pose[0, 3], pivot_pose[1, 3]

    # The tool tip's position when engaged, so the tip is what goes round the circle
    tip_pose = wdt_pose * build_pose([[1, 0, 0], [0, -1, 0], [0, 0, -1]], get_point(46))
    tip_x, tip_y = tip_pose[0, 3], tip_pose[1, 3]

    radius_mm = math.hypot(tip_x - pivot_x, tip_y - pivot_y)
    print(f"[task i] Orbit radius {radius_mm:.1f} mm")

    total_steps = int(num_revolutions * steps_per_revolution)
    steps = []
    for i in range(1, total_steps + 1):
        angle = spin_direction * 2 * pi * i / steps_per_revolution
        # Turns the tip's offset from the axis by this angle, then shifts the whole pose by how far the tip moved
        dx = tip_x - pivot_x
        dy = tip_y - pivot_y
        new_x = pivot_x + dx * math.cos(angle) - dy * math.sin(angle)
        new_y = pivot_y + dx * math.sin(angle) + dy * math.cos(angle)
        steps.append(transl(new_x - tip_x, new_y - tip_y, 0) * engaged_pose)

    return steps


def check_orbit_reachable(UR5, orbit_steps):
    # Reports whether each point on the orbit has an inverse kinematics solution,
    # so an unreachable point partway round is caught before running
    for i, step_pose in enumerate(orbit_steps, start=1):
        solution = UR5.SolveIK(step_pose)
        status = "reachable" if solution else "UNREACHABLE"
        print(f"[task i] Orbit step {i} of {len(orbit_steps)}, {status}")



def run_task_i(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================

    # fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the tool tip goes into the rotor hole, by shifting the assumed position of the wdt along the
    # wdt's own axes. The orbit moves with it.
    #   x: shifts along the wdt's local x
    #   y: shifts along the wdt's local y
    #   z: raises (+) or lowers (-) the tool, use this to control how deep the tip goes into the hole
    fixture_fine_tune_mm = (0, 0, 5)

    # approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z), pivoting about the tool tip so the tip
    # stays over the hole and only the tool's angle changes. Also fixes the tool's orientation for the whole
    # orbit, since the orientation does not turn as the tip goes round.
    approach_angle_adjust_deg = (0, 0, 135)

    # pivot_fine_tune_mm = (x, y, z) in mm
    # Shifts the centre the tip goes round, along the wdt's own axes. Only x and y matter, z does nothing.
    # It changes the radius of the circle, which starts at the hole's distance from the wdt frame origin.
    # Nudge it if the tool binds or drifts out of the hole part way round.
    pivot_fine_tune_mm = (0, 0, 0)

    # num_revolutions
    # How many full turns of the rotor, the assignment asks for 5.
    num_revolutions = 5

    # steps_per_revolution
    # How many points each turn is split into. More = a rounder path but a slower run and more moves,
    # fewer = faster but each move is a longer straight line that cuts inside the circle.
    steps_per_revolution = 24

    # spin_direction
    # 1 turns the tip anticlockwise seen from above, -1 turns it clockwise. Set it to whichever way the rotor
    # is meant to turn, if the tool pushes against the slot the wrong way, flip it.
    spin_direction = 1

    # align_hover_mm
    # How far straight above the hole (along the wdt's z) the tool lines up before dropping in, and lifts back to.
    # Bigger = more clearance, smaller = a shorter drop.
    align_hover_mm = 40

    # There are no jogged joint poses in this part, the joint move goes straight to the hover pose. If the arm hits
    # the wdt or the rancilio tool sitting in it on the way, jog a waypoint and add a MoveJ to it before the hover pose.
    # =============================================================================

    # Computes the pose lined up above the hole, the pose with the tip in the hole, and the points round the orbit
    task_i_hover_pose = compute_task_i_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, align_hover_mm)
    task_i_pose = compute_task_i_pose(fixture_fine_tune_mm, approach_angle_adjust_deg)
    orbit_steps = compute_task_i_orbit_steps(
        fixture_fine_tune_mm, approach_angle_adjust_deg, pivot_fine_tune_mm,
        num_revolutions, steps_per_revolution, spin_direction)

    # Run once with collision detection on to catch an unreachable point partway round
    # check_orbit_reachable(UR5, orbit_steps)

    # # Starts the robot at home
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)

    # Picks up the mazzer tool from the tool stand
    tls.mazzer_tool_attach_l_ati()

    # Moves in above the rotor hole, then drops the tip into it
    UR5.MoveJ(task_i_hover_pose, True)
    UR5.MoveL(task_i_pose, True)

    # Turns the rotor, the tip goes round the circle num_revolutions times and ends back where it started
    for step_pose in orbit_steps:
        UR5.MoveL(step_pose, True)

    # Lifts the tip out of the hole and retreats, then returns the mazzer tool
    UR5.MoveL(task_i_hover_pose, True)
    tls.mazzer_tool_detach_l_ati()

    # # Sends the robot home
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    run_task_i(UR5, tls, RDK)

    # Sends the robot home
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
