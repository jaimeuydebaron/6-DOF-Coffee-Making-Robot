# _partG.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, compute_waypoints, solve_frame_rotation


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


def compute_task_g_pose(fixture_fine_tune_mm, approach_angle_adjust_deg):
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

    return task_g_pose


# Jogged waypoints
away_general_point2 = [98.740000, -103.350000, 122.610000, -19.250000, 118.710000, 140.000000]
wdt_approach_joints = [126.734237, -98.713890, 145.107306, -46.393416, 36.734237, 140.000000]
wdt_pre_drop_joints = [142.649042, -84.551875, 131.581767, -47.029893, 52.649042, 140.000000]


def run_task_g(UR5, tls, RDK):
    # Edit these in the lab

    # Shifts the wdt's whole pose along its own local x, y, z directions to correct calibration drift
    fixture_fine_tune_mm = (12, 1, 12)

    # Rotates the tool's approach about its own x, y, z axes once it is already oriented at the placement point
    approach_angle_adjust_deg = (0, 0, 0)

    task_g_pose = compute_task_g_pose(fixture_fine_tune_mm, approach_angle_adjust_deg)

    waypoints = compute_waypoints(task_g_pose, offsets=[
        ("approach", (0, 0, -100), (0, 0, 0)),
    ])

    # The rancilio tool is already held, carried over from task f
    UR5.MoveJ(away_general_point2, True)

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

    ghost_tool = RDK.Item("Rancilio_Tool_(WDT)", ITEM_TYPE_OBJECT)
    ghost_tool.setVisible(True)

    tls.wdt_shut()

    # Retreats
    UR5.MoveL(waypoints["approach"], True)


def test_task_g(UR5, tls, RDK):
    run_task_g(UR5, tls, RDK)

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
    RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    test_task_g(UR5, tls, RDK)
