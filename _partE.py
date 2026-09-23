# _partE.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, rotate_about_tip, solve_frame_rotation


def compute_task_e_poses(fixture_fine_tune_mm, approach_angle_adjust_deg):
    # Slides the tool along the mazzer scale's own x, y, z directions before pressing the lever
    SLIDE_MM = (15, 0, 0)

    # Presses the tool further along the mazzer scale's own x, y, z directions to flip the lever back to locked
    PRESS_MM = (0, 0, -15)

    # Gets the mazzer scale's origin and a reference point, in both world and local coordinates
    origin_w = get_point(6)
    origin_l = get_point(27)
    ref_w = get_point(7)
    ref_l = get_point(28)

    # Solves for how far the mazzer scale is rotated relative to the world frame
    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    # Builds the mazzer scale's pose in the robot's world frame
    mazzer_scale_pose = transl(*origin_w) * rotz(theta)

    # Applies the fixture calibration correction
    mazzer_scale_pose = mazzer_scale_pose * transl(*fixture_fine_tune_mm)

    # Sets the placement point as the left lock lever, with the tool orientation found by inspection
    point_local = get_point(31)
    axis_mapping = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]

    # Sets the mazzer tool's contact point
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)

    # Builds the angle correction, rotated about the tool tip so its position doesn't move
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)

    # Builds the approach pose at the lock lever
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_scale_pose * T_fixture_placement
    task_e_lock_pose = rotate_about_tip(T_world_target * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # Builds the slide pose by offsetting the fixture pose along its own local frame
    mazzer_scale_pose_slid = mazzer_scale_pose * transl(*SLIDE_MM)
    T_world_target_slid = mazzer_scale_pose_slid * T_fixture_placement
    task_e_slide_pose = rotate_about_tip(T_world_target_slid * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # Builds the press pose by offsetting further along the fixture's own frame
    mazzer_scale_pose_pressed = mazzer_scale_pose_slid * transl(*PRESS_MM)
    T_world_target_pressed = mazzer_scale_pose_pressed * T_fixture_placement
    task_e_press_pose = rotate_about_tip(T_world_target_pressed * T_tool_offset.inv(), tool_point_local, angle_adjust)

    return task_e_lock_pose, task_e_slide_pose, task_e_press_pose


# Jogged joint waypoints for the lock approach and the locked config
lock_approach_joints = [100.175126, -73.098008, 118.468843, -104.907218, -35.456113, 14.167126]
lock_locked_joints = [100.764372, -69.791136, 116.900222, -107.365230, -35.160559, 15.049119]
staging_joints_general = [118.506308, -97.117079, 116.960032, -109.759306, -89.525329, -206.644986]


def run_task_e(UR5, tls, RDK):
    # Shifts the mazzer scale's whole pose along its own local x, y, z directions to correct calibration drift
    fixture_fine_tune_mm = (0, 0, 0)

    # Rotates the tool's approach about the tip, since the rancilio tool
    # will already be sitting on the mazzer scale when this task runs
    approach_angle_adjust_deg = (0, 60, -130)

    lock_pose, slide_pose, press_pose = compute_task_e_poses(fixture_fine_tune_mm, approach_angle_adjust_deg)

    # Picks up the mazzer tool from the tool stand
    tls.mazzer_tool_attach_l_ati()

    UR5.MoveJ(staging_joints_general, True)

    # Moves to the lock lever, slides, then presses to lock the scale
    UR5.MoveJ(lock_approach_joints, True)
    UR5.MoveL(lock_pose, True)
    UR5.MoveL(slide_pose, True)
    UR5.MoveL(press_pose, True)
    UR5.MoveL(lock_locked_joints, True)

    UR5.MoveJ(staging_joints_general, True)


def test_task_e(UR5, tls, RDK):
    run_task_e(UR5, tls, RDK)

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
    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    test_task_e(UR5, tls, RDK)
