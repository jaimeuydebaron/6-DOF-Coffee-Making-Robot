# _partGH.py
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import get_point, build_pose, solve_frame_rotation, run_visual, enable_space_pause

# needs to come out not upward but outward, needs to be angled slightly upward

def compute_wdt_pose(fixture_fine_tune_mm):
    # The wdt's frame, see part a for how this works
    origin_w = get_point(14)
    origin_l = get_point(44)
    ref_w = get_point(15)
    ref_l = get_point(45)

    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    wdt_pose = transl(*origin_w) * rotz(theta)

    wdt_pose = wdt_pose * transl(*fixture_fine_tune_mm)

    return wdt_pose


def compute_task_g_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg=0):
    wdt_pose = compute_wdt_pose(fixture_fine_tune_mm)

    # The target is the plate centre drill hole (point 45)
    point_local = get_point(45)
    axis_mapping = [
        [0, 0, 1],   # Tool x axis lines up with the fixture's z axis
        [0, 1, 0],   # Tool y axis lines up with the fixture's y axis
        [-1, 0, 0],  # Tool z axis lines up with the fixture's negative x axis
    ]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = wdt_pose * T_fixture_placement

    # The tool's contact point is its ball bearing indent (point 54)
    tool_point_local = get_point(54)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    task_g_pose = T_world_target * T_tool_offset.inv()

    # Angle adjust about the tcp
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_g_pose = task_g_pose * angle_adjust

    # Tilts about the tool's own y (not the tcp's, the tool is mounted rotated) by the stem angle plus
    # extra_tilt_deg, pivoting about point 54. T_tool_offset moves into the tool's frame at that point
    tilt_deg = get_point(63)[0] + extra_tilt_deg
    task_g_pose = task_g_pose * T_tool_offset * roty(tilt_deg * pi/180) * T_tool_offset.inv()

    return task_g_pose



# Jogged waypoints
approach_intermediate_pose_1 = [147.869438, -84.061570, 135.628719, -52.818138, 119.138938, -220.596797]
wdt_approach_joints = [126.734237, -98.713890, 145.107306, -46.393416, 36.734237, -220.000000]
wdt_pre_drop_joints = [143.051468, -82.077554, 133.714103, -56.263009, 53.141746, -217.220931]

retreat_intermediate_pose_1 = [136.540486, -80.982847, 143.019227, -67.127780, 46.654059, 143.499694]
retreat_intermediate_pose_2 = [118.916783, -94.293217, 121.650141, -117.276423, -89.524465, -206.234482]


def run_task_g(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================

    # Shifts where the tool is placed in the wdt, along the wdt's own axes
    fixture_fine_tune_mm = (8, 0, 7)

    # Rotates the tool about the tcp, so the placement moves too, re-tune the fine tune after changing it
    approach_angle_adjust_deg = (0, 0, 0)

    # Added to the stem angle, pivoting about point 54. The cup centre is about 120 mm from the pivot, so it
    # moves about 2 mm per degree
    extra_tilt_deg = 0

    # =============================================================================

    task_g_pose = compute_task_g_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg)

    # The rancilio tool is already held, carried over from task f
    UR5.MoveJ(approach_intermediate_pose_1, True)

    # Opens the wdt fixture before placing the tool
    tls.wdt_open()

    # Moves in and places the rancilio tool into the wdt
    UR5.MoveL(wdt_approach_joints, True)
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

    # Picks up the rancilio tool, in the full run it is already held, carried over from task f
    tls.rancilio_tool_attach_l_ati()

    run_task_g(UR5, tls, RDK)

    # Sends the robot home
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
