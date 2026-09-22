# _partD.py
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


def normalize(v):
    length = math.sqrt(sum(c**2 for c in v))
    return tuple(c / length for c in v)


def cross_product(a, b):
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    )


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


def compute_mazzer_pose():
    # gets the mazzer origin and two reference points, one on the local x axis and one on the local y axis
    origin_w = get_point(3)
    x_ref_w = get_point(4)
    y_ref_w = get_point(5)

    # finds the direction from the origin to each reference point in world coordinates
    x_dir = tuple(x_ref_w[i] - origin_w[i] for i in range(3))
    y_dir = tuple(y_ref_w[i] - origin_w[i] for i in range(3))

    # normalizes both directions into unit vectors, these become the x and y columns of the rotation matrix
    x_axis = normalize(x_dir)
    y_axis = normalize(y_dir)
    x_axis = tuple(-c for c in x_axis)

    # computes the z axis as the cross product to keep the frame right handed
    z_axis = cross_product(x_axis, y_axis)

    mazzer_pose = build_pose([x_axis, y_axis, z_axis], origin_w)
    return mazzer_pose


def compute_task_d_pose(fixture_fine_tune_mm, approach_angle_adjust_deg):
    mazzer_pose = compute_mazzer_pose()
    mazzer_pose = mazzer_pose * transl(*fixture_fine_tune_mm)

    # sets the placement point as the dose lever face and the tool's orientation there
    point_local = get_point(24)
    axis_mapping = [[0,1,0], [0,0,-1], [-1,0,0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    # sets the mazzer tool's contact point, the dose bar edge is the pulling surface
    tool_point_local = get_point(52)
    T_tool_offset = transl(*tool_point_local)
    task_d_pose = T_world_target * T_tool_offset.inv()

    # applies the approach angle correction
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_d_pose = task_d_pose * angle_adjust

    return task_d_pose


def run_task_d(UR5, tls, RDK):
    # edit these in the lab

    # shifts the mazzer's whole pose in its own local x, y, z directions to correct calibration drift
    fixture_fine_tune_mm = (0, 0, 0)

    # rotates the tool's approach about its own x, y, z axes once already oriented at the lever
    approach_angle_adjust_deg = (0, 0, 0)

    # how far to pull the dose lever, in the tool's own local frame, per pull
    # this is a measured/tuned value, not derived from the points spreadsheet,
    # since the assignment brief expects grind weight to be checked and refined
    # against the scale reading, not calculated in advance
    pull_distance_mm = (0, 0, -20)

    task_d_pose = compute_task_d_pose(fixture_fine_tune_mm, approach_angle_adjust_deg)

    waypoints = compute_waypoints(task_d_pose, offsets=[
        ("approach", (0, 0, -50), (0, 0, 0)),
        ("pulled", pull_distance_mm, (0, 0, 0)),
    ])

    # picks up the mazzer tool from the tool stand
    tls.mazzer_tool_attach_r_ati()

    # moves in, pulls the lever, retreats
    UR5.MoveJ(waypoints["approach"], True)
    UR5.MoveL(task_d_pose, True)
    UR5.MoveL(waypoints["pulled"], True)

    # note: the assignment brief requires repeating this pull until the mazzer
    # scale reports 20 plus or minus 0.1g deposited in the rancilio tool, this
    # needs a scale-reading check wired in here once that interface is confirmed

    UR5.MoveL(task_d_pose, True)
    UR5.MoveL(waypoints["approach"], True)

    tls.mazzer_tool_detach_r_ati()
    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)


if __name__ == "__main__":
    RDK = Robolink()

    # --- simulation settings (use when testing in robodk only) ---
    RDK.setRunMode(RUNMODE_SIMULATE)

    # --- real robot settings (uncomment when running on the physical robot) ---
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # connect to the robot before running: connect -> connect robot, then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    run_task_d(UR5, tls, RDK)