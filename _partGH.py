# _partG.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, compute_waypoints, solve_frame_rotation, rotate_about_tip, run_visual, enable_step_mode

# needs to come out not upward but outward, needs to be angled slightly upward

def compute_wdt_pose(fixture_fine_tune_mm):
    # Gets the wdt's origin and a reference point, in both world and local coordinates
    origin_w = get_point(14)
    origin_l = get_point(44)
    ref_w = get_point(15)
    ref_l = get_point(45)

    # Solves for how far the wdt is rotated relative to the world frame
    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    # Builds the wdt's pose in the robot's world frame
    wdt_pose = transl(*origin_w) * rotz(theta)

    # Applies the fixture calibration correction
    wdt_pose = wdt_pose * transl(*fixture_fine_tune_mm)

    return wdt_pose


def compute_task_g_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg=0):
    wdt_pose = compute_wdt_pose(fixture_fine_tune_mm)

    # Sets the placement point as the plate centre drill hole, with the tool orientation found by inspection
    point_local = get_point(45)
    axis_mapping = [
        [0, 0, 1],   # Tool x axis lines up with the fixture's z axis
        [0, 1, 0],   # Tool y axis lines up with the fixture's y axis
        [-1, 0, 0],  # Tool z axis lines up with the fixture's negative x axis
    ]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = wdt_pose * T_fixture_placement

    # Sets the rancilio tool's contact point and its mounting angle offset
    tool_point_local = get_point(54)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    task_g_pose = T_world_target * T_tool_offset.inv()

    # Applies the approach angle correction
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_g_pose = task_g_pose * angle_adjust

    # Tilts the tool positively about the TOOL's own y (not the flange's y, the tool is mounted rotated relative
    # to the flange) by the stem angle (point 63) plus extra_tilt_deg, pivoting about the ball bearing point
    # (point 54) so it stays on the placement point. T_tool_offset carries the tool frame from the flange to
    # point 54, so rotating in that frame is a rotation about the tool's y through point 54.
    tilt_deg = get_point(63)[0] + extra_tilt_deg
    task_g_pose = task_g_pose * T_tool_offset * roty(tilt_deg * pi/180) * T_tool_offset.inv()

    return task_g_pose



# Jogged waypoints
approach_intermediate_pose_1 = [147.869438, -84.061570, 135.628719, -52.818138, 119.138938, 139.403203]
wdt_approach_joints = [126.734237, -98.713890, 145.107306, -46.393416, 36.734237, 140.000000]
wdt_pre_drop_joints = [143.051468, -82.077554, 133.714103, -56.263009, 53.141746, 142.779069]

retreat_intermediate_pose_1 = [136.540486, -80.982847, 143.019227, -67.127780, 46.654059, 143.499694]
retreat_intermediate_pose_2 = [118.916783, -94.293217, 121.650141, -117.276423, -89.524465, -206.234482]


def run_task_g(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================

    # fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the tool is placed in the wdt, by shifting the assumed position of the wdt along the wdt's
    # own axes. The tool keeps its angle and just moves as a whole.
    #   x: shifts along the wdt's local x
    #   y: shifts along the wdt's local y
    #   z: shifts along the wdt's local z
    # Use it when the tool misses the drill hole in the plate centre, then check which way it needs to move.
    fixture_fine_tune_mm = (8, 0, 7)

    # approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z) at the placement pose.
    # It pivots about the tool's origin, NOT the placement point, so changing it also moves where the tool lands.
    # Change the angle first, then re-tune fixture_fine_tune_mm to put the tool back on the drill hole.
    # The comment at the top of this file says the tool should leave outward and slightly upward, not straight up,
    # so this is the knob to use for that angle.
    approach_angle_adjust_deg = (0, 0, 0)

    # extra_tilt_deg
    # Extra lean added on top of the 1.7 degree stem angle (point 63), about the tool's own y, pivoting about the
    # ball bearing point (point 54) so it stays where it is and the rest of the tool swings around it.
    # Applied after approach_angle_adjust_deg. Positive = leans further, negative = reduces the tilt
    # (-1.7 gives no tilt). Note the cup centre is about 120 mm from the pivot, so it moves about 2 mm per degree.
    extra_tilt_deg = 0

    # The jogged joint poses (away_general_point2, wdt_approach_joints, wdt_pre_drop_joints) are NOT affected by
    # the settings above. wdt_pre_drop_joints is the standoff the tool moves in from before the final MoveL to
    # task_g_pose, so if you move the placement a lot, re-jog it so that last straight move stays clear of the wdt.
    # =============================================================================

    task_g_pose = compute_task_g_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg)

    # Optional, builds extra waypoints from the final tilted pose instead of using align_hover_mm
    # Each offset is (name, (x, y, z) in mm, (rx, ry, rz) in degrees), applied in the tool's OWN frame,
    # translation first then rotation. Each waypoint comes from task_a_pose on its own, they do not chain.
    # For example, this backs the tool off 40 mm along its own z, keeping the tilt:
    waypoints = compute_waypoints(task_g_pose, offsets=[
        ("backoff", (0, 0, -50), (0, 0, 0)),
    ])
    # UR5.MoveL(waypoints["backoff"], True)
    # UR5.MoveL(task_a_pose, True)

    # The rancilio tool is already held, carried over from task f
    UR5.MoveJ(approach_intermediate_pose_1, True)

    # Opens the wdt fixture before placing the tool
    tls.wdt_open()

    # Moves in and places the rancilio tool into the wdt
    UR5.MoveJ(wdt_approach_joints, True)
    UR5.MoveL(wdt_pre_drop_joints, True)
    UR5.MoveL(task_g_pose, True)

    # Releases the tool, shows it sitting in the wdt fixture, then closes the wdt in task h
    tls.student_tool_detach()

    attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
    attached_tool.setVisible(False, False)
    run_visual(RDK, "Show_WDT_Rancilio_Tool")

    UR5.MoveL(retreat_intermediate_pose_1, True)

    tls.wdt_shut()

    UR5.MoveJ(wdt_approach_joints, True)
    UR5.MoveJ(retreat_intermediate_pose_2, True)

    # Sends the robot home
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)

    # Waits for enter before every move, for testing this task on its own, comment out to run straight through
    enable_step_mode(UR5, tls)

    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # Picks up the rancilio tool, in the full run it is already held, carried over from task f
    tls.rancilio_tool_attach_l_ati()

    run_task_g(UR5, tls, RDK)
