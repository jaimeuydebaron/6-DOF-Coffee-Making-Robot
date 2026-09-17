# _partB.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from __points import POINTS
import math


def get_point(key):
    entry = POINTS[key]
    return entry[3], entry[4], entry[5]


def build_pose(rotation_columns, xyz):
    col_x, col_y, col_z = rotation_columns
    pose = eye(4)
    for i in range(3):
        pose[i, 0] = col_x[i]
        pose[i, 1] = col_y[i]
        pose[i, 2] = col_z[i]
    pose[0, 3], pose[1, 3], pose[2, 3] = xyz
    return pose


def clean_pose(pose):
    good_ht = UR_2_Pose(Pose_2_UR(pose))
    return good_ht


def compute_task_b_poses():
    # shifts the mazzer scale's whole pose in its own local x, y, z directions to correct calibration drift
    FIXTURE_FINE_TUNE_MM = (0, 0, 0)

    # slides the tool along the mazzer scale's own x, y, z directions before pressing the lever
    SLIDE_MM = (-20, 0, 0)

    # presses the tool further along the mazzer scale's own x, y, z directions to flip the lever
    PRESS_MM = (0, 0, -15)

    # rotates the mazzer tool's contact point about its own z axis
    TOOL_MOUNT_ANGLE_DEG = 130

    # gets the mazzer scale origin and reference point in world and local coordinates
    origin_w = get_point(6)
    origin_l = get_point(27)
    ref_w = get_point(7)
    ref_l = get_point(28)

    # finds the displacement between origin and reference point in both frames
    world_dx = ref_w[0] - origin_w[0]
    world_dy = ref_w[1] - origin_w[1]
    local_dx = ref_l[0] - origin_l[0]
    local_dy = ref_l[1] - origin_l[1]

    # solves for the rotation angle between the two frames
    cross = local_dx * world_dy - local_dy * world_dx
    dot   = local_dx * world_dx + local_dy * world_dy
    theta = math.atan2(cross, dot)

    # builds the mazzer scale's pose in the robot's world frame
    mazzer_scale_pose = transl(*origin_w) * rotz(theta)

    # applies the fixture calibration correction
    mazzer_scale_pose = mazzer_scale_pose * transl(*FIXTURE_FINE_TUNE_MM)

    # sets the placement point as the left lock lever and the tool's orientation there
    point_local = get_point(30)
    axis_mapping = [[0,1,0], [1,0,0], [0,0,-1]]

    # sets the mazzer tool's contact point and mounting angle offset
    tool_point_local = get_point(51)
    T_tool_offset = rotz(TOOL_MOUNT_ANGLE_DEG * pi/180) * transl(*tool_point_local)

    # builds the approach pose at the lock lever
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_scale_pose * T_fixture_placement
    task_b_approach_pose = T_world_target * T_tool_offset.inv()

    # builds the slide pose by offsetting the fixture pose along its own local frame
    mazzer_scale_pose_slid = mazzer_scale_pose * transl(*SLIDE_MM)
    T_world_target_slid = mazzer_scale_pose_slid * T_fixture_placement
    task_b_slide_pose = T_world_target_slid * T_tool_offset.inv()

    # builds the press pose by offsetting further along the fixture's own frame
    mazzer_scale_pose_pressed = mazzer_scale_pose_slid * transl(*PRESS_MM)
    T_world_target_pressed = mazzer_scale_pose_pressed * T_fixture_placement
    task_b_press_pose = T_world_target_pressed * T_tool_offset.inv()

    return task_b_approach_pose, task_b_slide_pose, task_b_press_pose


# intermediate pose to avoid hitting mazzer
staging_pose_1 = Mat([
    [-0.939853,  0.341580,  0.000012, 415.802930],
    [ 0.341580,  0.939853, -0.000002, -189.997992],
    [-0.000012,  0.000002, -1.000000, 169.519942],
    [ 0.000000,  0.000000,  0.000000,   1.000000],
])

# linear move from previous intermediate point to just above the on pose
staging_pose_2 = Mat([
    [-0.939852,  0.341581, -0.000030, 475.802944],
    [ 0.341581,  0.939852,  0.000005, -289.997977],
    [ 0.000030, -0.000005, -1.000000, 169.519944],
    [ 0.000000,  0.000000,  0.000000,   1.000000],
])


def run_task_b(UR5, tls, RDK):
    approach_pose, slide_pose, press_pose = compute_task_b_poses()

    # picks up the mazzer tool from the tool stand
    # tls.mazzer_tool_attach_r_ati()

    # # moves through the two staging poses to avoid colliding with the coffee machine
    # UR5.MoveJ(staging_pose_1, True)
    # UR5.MoveL(clean_pose(staging_pose_2), True)

    # # moves to the lock lever, slides, then presses to unlock the scale
    UR5.MoveL(approach_pose, True)
    UR5.MoveL(slide_pose, True)
    UR5.MoveL(press_pose, True)

    # # returns the mazzer tool and sends the robot home
    # tls.mazzer_tool_detach_r_ati()
    # UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)


if __name__ == "__main__":
    RDK = Robolink()

    # --- SIMULATION SETTINGS (use when testing in RoboDK only) ---
    RDK.setRunMode(RUNMODE_SIMULATE)

    # --- REAL ROBOT SETTINGS (uncomment when running on the physical robot) ---
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # connect to the robot before running: Connect -> Connect robot, then press Connect and Get Position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    # RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM).RunCode()

    run_task_b(UR5, tls, RDK)