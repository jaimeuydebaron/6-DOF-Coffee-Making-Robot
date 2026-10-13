# _partI.py
# Task i, use the mazzer tool to turn the wdt rotor five full revolutions
from robodk.robolink import *
from robodk.robomath import *
import math
import tools
from _common import get_point, build_pose, rotate_about_tip, run_visual, enable_space_pause
from _partGH import compute_wdt_pose


def compute_task_i_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm=0):
    # Same steps as part a, on the wdt's frame
    wdt_pose = compute_wdt_pose(fixture_fine_tune_mm)

    # The target is the rotor fastener hole (point 46), with the tool pointing straight down into it
    point_local = get_point(46)
    axis_mapping = [
        [1, 0, 0],   # Tool x axis lines up with the fixture's x axis
        [0, -1, 0],  # Tool y axis lines up with the fixture's negative y axis
        [0, 0, -1],  # Tool z axis lines up with the fixture's negative z axis, pointing down into the rotor
    ]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    # Raised along the wdt's z by hover_mm
    T_world_target = wdt_pose * transl(0, 0, hover_mm) * T_fixture_placement

    # The tool's contact point is its tip (point 51)
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    task_i_pose = T_world_target * T_tool_offset.inv()

    # Angle adjust about the tip
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_i_pose = rotate_about_tip(task_i_pose, tool_point_local, angle_adjust)

    return task_i_pose


def compute_task_i_orbit_steps(fixture_fine_tune_mm, approach_angle_adjust_deg, pivot_fine_tune_mm,
                               num_revolutions, steps_per_revolution, spin_direction):
    # The tip goes round a circle centred on the rotor's axis while the tool keeps the same orientation. The
    # rotor turns because the tip sits in an off centre hole, like a crank, and turning the tool itself five
    # times would wind the wrist past its limit
    wdt_pose = compute_wdt_pose(fixture_fine_tune_mm)
    engaged_pose = compute_task_i_pose(fixture_fine_tune_mm, approach_angle_adjust_deg)

    # The rotor axis runs along the wdt's z through the wdt frame origin, shifted by pivot_fine_tune_mm
    pivot_pose = wdt_pose * transl(*pivot_fine_tune_mm)
    pivot_x, pivot_y = pivot_pose[0, 3], pivot_pose[1, 3]

    # The tip's position when engaged
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
    # Prints whether each point on the orbit has an inverse kinematics solution
    for i, step_pose in enumerate(orbit_steps, start=1):
        solution = UR5.SolveIK(step_pose)
        status = "reachable" if solution else "UNREACHABLE"
        print(f"[task i] Orbit step {i} of {len(orbit_steps)}, {status}")



def run_task_i(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================

    # Shifts where the tip goes into the rotor hole, along the wdt's own axes. z sets how deep it goes
    fixture_fine_tune_mm = (0, 0, 5)

    # Rotates the tool about its tip, and sets the orientation it keeps for the whole orbit
    approach_angle_adjust_deg = (0, 0, 135)

    # Shifts the centre of the orbit along the wdt's x and y, nudge it if the tool binds or drifts out of the hole
    pivot_fine_tune_mm = (0, 0, 0)

    # Full turns of the rotor, the assignment asks for 5
    num_revolutions = 5

    # Points per turn, more is rounder but slower
    steps_per_revolution = 24

    # 1 turns anticlockwise seen from above, -1 clockwise
    spin_direction = 1

    # Height above the hole where the tool lines up before dropping in, and lifts back to
    align_hover_mm = 40

    # No jogged poses here, add a MoveJ before the hover pose if the arm hits the wdt on the way
    # =============================================================================

    # The pose above the hole, the pose with the tip in it, and the points round the orbit
    task_i_hover_pose = compute_task_i_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, align_hover_mm)
    task_i_pose = compute_task_i_pose(fixture_fine_tune_mm, approach_angle_adjust_deg)
    orbit_steps = compute_task_i_orbit_steps(
        fixture_fine_tune_mm, approach_angle_adjust_deg, pivot_fine_tune_mm,
        num_revolutions, steps_per_revolution, spin_direction)

    # Run once with collision detection on to catch an unreachable point partway round
    # check_orbit_reachable(UR5, orbit_steps)

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


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)

    # Space in the terminal pauses and plays the run, for testing this task on its own
    enable_space_pause(UR5, tls)

    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # Shows the rancilio tool sitting in the wdt and takes it off the tool stand, as tasks g and h leave it.
    # Simulation only
    run_visual(RDK, "Show_WDT_Rancilio_Tool")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    # Starts at home, where tasks g and h finish
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)

    run_task_i(UR5, tls, RDK)

    # Sends the robot home, in the full run task j carries straight on from the tool stand
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
