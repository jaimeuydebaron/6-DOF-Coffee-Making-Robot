# _partD.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, compute_waypoints, compute_mazzer_pose


def compute_task_d_pose(fixture_fine_tune_mm, approach_angle_adjust_deg):
    mazzer_pose = compute_mazzer_pose()
    mazzer_pose = mazzer_pose * transl(*fixture_fine_tune_mm)

    # Sets the placement point as the dose lever face, with the tool orientation found by inspection
    point_local = get_point(24)
    axis_mapping = [[0,1,0], [0,0,-1], [-1,0,0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    # Sets the mazzer tool's contact point, the dose bar edge is the pulling surface
    tool_point_local = get_point(52)
    T_tool_offset = transl(*tool_point_local)
    task_d_pose = T_world_target * T_tool_offset.inv()

    # Applies the approach angle correction
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_d_pose = task_d_pose * angle_adjust

    return task_d_pose


def run_task_d(UR5, tls, RDK):
    # Edit these in the lab

    # Shifts the mazzer's whole pose along its own local x, y, z directions to correct calibration drift
    fixture_fine_tune_mm = (0, 0, 0)

    # Rotates the tool's approach about its own x, y, z axes once it is already oriented at the lever
    approach_angle_adjust_deg = (0, 0, 0)

    # How far to pull the dose lever, in the tool's own local frame, per pull
    # This is a measured and tuned value rather than one derived from the points spreadsheet,
    # since the assignment brief expects grind weight to be checked and refined
    # against the scale reading rather than calculated in advance
    pull_distance_mm = (0, 0, -20)

    task_d_pose = compute_task_d_pose(fixture_fine_tune_mm, approach_angle_adjust_deg)

    waypoints = compute_waypoints(task_d_pose, offsets=[
        ("approach", (0, 0, -50), (0, 0, 0)),
        ("pulled", pull_distance_mm, (0, 0, 0)),
    ])

    # Picks up the mazzer tool from the tool stand
    tls.mazzer_tool_attach_r_ati()

    # Moves in, pulls the lever, retreats
    UR5.MoveJ(waypoints["approach"], True)
    UR5.MoveL(task_d_pose, True)
    UR5.MoveL(waypoints["pulled"], True)

    # Note, the assignment brief requires repeating this pull until the mazzer
    # scale reports 20 grams plus or minus 0.1 grams deposited in the rancilio tool,
    # this needs a scale reading check wired in here once that interface is confirmed

    UR5.MoveL(task_d_pose, True)
    UR5.MoveL(waypoints["approach"], True)

    tls.mazzer_tool_detach_r_ati()


def test_task_d(UR5, tls, RDK):
    run_task_d(UR5, tls, RDK)

    # Sends the robot home
    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)


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

    test_task_d(UR5, tls, RDK)
