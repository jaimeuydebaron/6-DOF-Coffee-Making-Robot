# _partC.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, compute_mazzer_pose


def compute_task_c_on_pose(fixture_fine_tune_mm, approach_angle_adjust_deg):
    mazzer_pose = compute_mazzer_pose()
    mazzer_pose = mazzer_pose * transl(*fixture_fine_tune_mm)

    # Sets the placement point as the on button, with the tool orientation found by inspection
    point_local = get_point(26)
    axis_mapping = [[0, 1, 0], [0, 0, -1], [-1, 0, 0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    # Sets the mazzer tool's contact point
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    task_c_on_pose = T_world_target * T_tool_offset.inv()

    # Applies the approach angle correction
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_c_on_pose = task_c_on_pose * angle_adjust

    return task_c_on_pose


def compute_task_c_off_pose(fixture_fine_tune_mm, approach_angle_adjust_deg):
    mazzer_pose = compute_mazzer_pose()
    mazzer_pose = mazzer_pose * transl(*fixture_fine_tune_mm)

    # Sets the placement point as the off button, with the tool orientation found by inspection
    point_local = get_point(25)
    axis_mapping = [[0, 1, 0], [0, 0, -1], [-1, 0, 0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    task_c_off_pose = T_world_target * T_tool_offset.inv()

    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_c_off_pose = task_c_off_pose * angle_adjust

    return task_c_off_pose


# Jogged pose that clears the mazzer body on the way in from home
staging_pose = Mat([
    [ 0.576111,  0.817367, -0.002682, 172.306970],
    [ 0.817338, -0.576114, -0.007157, -409.750946],
    [-0.007395,  0.001931, -0.999971, 176.463932],
    [ 0.000000,  0.000000,  0.000000,   1.000000],
])

# Joint waypoint that ensures the on button is approached at the right angle
on_button_approach_pose = [99.408373, -61.514148, 105.709453, -40.390944, 22.650255, 105.236545]

# Joint waypoint that ensures the off button is approached at the right angle
off_button_approach_pose = [99.693733, -59.564093, 102.529120, -39.203535, 22.933805, 105.284723]


def run_task_c(UR5, tls, RDK):
    # Edit these in the lab

    # Shifts the mazzer's whole pose along its own local x, y, z directions to correct calibration drift for the on button
    on_fixture_fine_tune_mm = (-32, -20, 50)
    # Rotates the tool's approach about its own x, y, z axes once it is already oriented at the on button
    on_approach_angle_adjust_deg = (-40, 15, 0)

    # Shifts the mazzer's whole pose along its own local x, y, z directions to correct calibration drift for the off button
    off_fixture_fine_tune_mm = (-32, -20, 50)
    # Rotates the tool's approach about its own x, y, z axes once it is already oriented at the off button
    off_approach_angle_adjust_deg = (-40, 15, 0)

    # Computes the final poses for pressing the on and off buttons
    on_pose = compute_task_c_on_pose(on_fixture_fine_tune_mm, on_approach_angle_adjust_deg)
    off_pose = compute_task_c_off_pose(off_fixture_fine_tune_mm, off_approach_angle_adjust_deg)

    # Picks up the mazzer tool from the tool stand
    tls.mazzer_tool_attach_l_ati()

    # Moves in, through a jogged joint waypoint, to press the on button
    UR5.MoveJ(staging_pose, True)
    UR5.MoveJ(on_button_approach_pose, True)

    sleep(3)  # Grind time, currently a placeholder for testing rather than the real brew length

    # Moves to press the off button, then through a jogged joint waypoint to retreat safely
    UR5.MoveL(off_pose, True)
    UR5.MoveL(off_button_approach_pose, True)

    # Retreats and returns the mazzer tool
    UR5.MoveJ(staging_pose, True)
    tls.mazzer_tool_detach_l_ati()


def test_task_c(UR5, tls, RDK):
    run_task_c(UR5, tls, RDK)

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

    test_task_c(UR5, tls, RDK)
