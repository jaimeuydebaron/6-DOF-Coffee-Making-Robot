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


def compute_waypoints(target_pose, offsets):
    # builds a dict of named waypoints, each derived from target_pose by
    # a translation and rotation offset in the tool's own local frame
    # offsets: list of (name, (x_mm, y_mm, z_mm), (rx_deg, ry_deg, rz_deg))
    waypoints = {}

    for name, translation_mm, angle_deg in offsets:
        angle_offset = rotx(angle_deg[0]*pi/180) \
                     * roty(angle_deg[1]*pi/180) \
                     * rotz(angle_deg[2]*pi/180)
        waypoints[name] = target_pose * transl(*translation_mm) * angle_offset

    return waypoints


def rotate_about_tip(pose, tip_offset_mm, angle_adjust):
    # rotates a pose about the tool's tip instead of about the tcp, so the
    # tip's position stays fixed while only its facing direction changes
    return pose \
        * transl(*tip_offset_mm) \
        * angle_adjust \
        * transl(*tuple(-c for c in tip_offset_mm))


def compute_task_b_poses(fixture_fine_tune_mm, approach_angle_adjust_deg):
    # slides the tool along the mazzer scale's own x, y, z directions before pressing the lever
    SLIDE_MM = (15, 0, 0)

    # presses the tool further along the mazzer scale's own x, y, z directions to flip the lever
    PRESS_MM = (0, 0, -15)

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
    mazzer_scale_pose = mazzer_scale_pose * transl(*fixture_fine_tune_mm)

    # sets the placement point as the left lock lever and the tool's orientation there
    point_local = get_point(31)
    axis_mapping = [[0,1,0], [1,0,0], [0,0,-1]]

    # sets the mazzer tool's contact point
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)

    # angle correction, rotated about the tool tip so its position doesn't move
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)

    # builds the approach pose at the lock lever
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_scale_pose * T_fixture_placement
    task_b_lock_pose = rotate_about_tip(T_world_target * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # builds the slide pose by offsetting the fixture pose along its own local frame
    mazzer_scale_pose_slid = mazzer_scale_pose * transl(*SLIDE_MM)
    T_world_target_slid = mazzer_scale_pose_slid * T_fixture_placement
    task_b_slide_pose = rotate_about_tip(T_world_target_slid * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # builds the press pose by offsetting further along the fixture's own frame
    mazzer_scale_pose_pressed = mazzer_scale_pose_slid * transl(*PRESS_MM)
    T_world_target_pressed = mazzer_scale_pose_pressed * T_fixture_placement
    task_b_press_pose = rotate_about_tip(T_world_target_pressed * T_tool_offset.inv(), tool_point_local, angle_adjust)

    return task_b_lock_pose, task_b_slide_pose, task_b_press_pose


# lock position approach pose before unlocking to avoid hitting rancilio tool
unlock_approach_joints = Mat([
    [100.175126],
    [-73.098008],
    [118.468843],
    [-104.907218],
    [-35.456113],
    [14.167126],
])

unlock_locked_joints = [100.764372, -69.791136, 116.900222, -107.365230, -35.160559, 15.049119]

staging_joints_general = Mat([
    [118.506308],
    [-97.117079],
    [116.960032],
    [-109.759306],
    [-89.525329],
    [-206.644986],
])


def run_task_b(UR5, tls, RDK):
    # edit these in the lab

    # shifts the mazzer scale's whole pose in its own local x, y, z directions to correct calibration drift
    fixture_fine_tune_mm = (0, 0, 0)

    # rotates the tool's approach about the tip, since the rancilio tool
    # will already be sitting on the mazzer scale when this task runs
    approach_angle_adjust_deg = (0, 60, -130)

    unlock_pose, slide_pose, press_pose = compute_task_b_poses(fixture_fine_tune_mm, approach_angle_adjust_deg)

    # compute a waypoint
    unlock_waypoint = compute_waypoints(unlock_pose, offsets=[("approach", (0, 0, -50), (0, 0, 0))])
    lock_waypoint = compute_waypoints(press_pose, offsets=[("approach", (0, 0, -50), (0, 0, 0))])

    # picks up the mazzer tool from the tool stand
    tls.mazzer_tool_attach_l_ati()

    UR5.MoveJ(staging_joints_general, True)

    # # moves to the lock lever, slides, then presses to unlock the scale
    # UR5.MoveJ(unlock_waypoint["approach"], True)
    UR5.MoveJ(unlock_approach_joints, True)
    UR5.MoveL(unlock_pose, True)
    UR5.MoveL(slide_pose, True)
    UR5.MoveL(press_pose, True)
    # UR5.MoveL(lock_waypoint["approach"], True)
    UR5.MoveL(unlock_locked_joints, True)

    UR5.MoveJ(staging_joints_general, True)


    # # returns the mazzer tool and sends the robot home
    # tls.mazzer_tool_detach_l_ati()
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)


if __name__ == "__main__":
    RDK = Robolink()

    # --- simulation settings (use when testing in robodk only) ---
    RDK.setRunMode(RUNMODE_SIMULATE)

    # --- real robot settings (uncomment when running on the physical robot) ---
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # connect to the robot before running: connect -> connect robot, then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    run_task_b(UR5, tls, RDK)