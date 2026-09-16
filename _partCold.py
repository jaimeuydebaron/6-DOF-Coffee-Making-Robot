# _partC.py
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


def clean_pose(pose):
    good_ht = UR_2_Pose(Pose_2_UR(pose))
    return good_ht


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


def compute_task_c_pose():
    # edit these values in the lab

    # shifts the mazzer's whole pose in its own local x, y, z directions to correct calibration drift
    FIXTURE_FINE_TUNE_MM = (0, 0, 0)

    # rotates the tool's approach about its own x, y, z axes once already oriented at the button
    APPROACH_ANGLE_ADJUST_DEG = (0, 0, 0)

    mazzer_pose = compute_mazzer_pose()
    mazzer_pose = mazzer_pose * transl(*FIXTURE_FINE_TUNE_MM)

    # sets the placement point as the on button and the tool's orientation there
    point_local = get_point(26)
    axis_mapping = [[0,1,0], [0,0,-1], [-1,0,0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    # sets the mazzer tool's contact point and mounting angle offset
    tool_point_local = get_point(51)
    T_tool_offset = rotz(0 * pi/180) * transl(*tool_point_local)
    task_c_pose = T_world_target * T_tool_offset.inv()

    # applies the approach angle correction
    angle_adjust = rotx(APPROACH_ANGLE_ADJUST_DEG[0]*pi/180) \
                 * roty(APPROACH_ANGLE_ADJUST_DEG[1]*pi/180) \
                 * rotz(APPROACH_ANGLE_ADJUST_DEG[2]*pi/180)
    task_c_pose = task_c_pose * angle_adjust

    return task_c_pose


def compute_task_c_off_pose():
    # edit these values in the lab
    FIXTURE_FINE_TUNE_MM = (0, 0, 0)

    mazzer_pose = compute_mazzer_pose()
    mazzer_pose = mazzer_pose * transl(*FIXTURE_FINE_TUNE_MM)

    # sets the placement point as the off button and the tool's orientation there
    point_local = get_point(25)
    axis_mapping = [[0,1,0], [0,0,-1], [-1,0,0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    tool_point_local = get_point(51)
    T_tool_offset = rotz(0 * pi/180) * transl(*tool_point_local)
    return T_world_target * T_tool_offset.inv()


# jogged pose that clears the mazzer body on the way in from home
staging_pose = Mat([
    [ 0.576111,  0.817367, -0.002682, 172.306970],
    [ 0.817338, -0.576114, -0.007157, -409.750946],
    [-0.007395,  0.001931, -0.999971, 176.463932],
    [ 0.000000,  0.000000,  0.000000,   1.000000],
])

# same target pose as compute_task_c_pose but expressed as a Cartesian pose, kept for reference
approach_pose_cartesian = Mat([
    [ 0.009637,  0.223092,  0.974750, 327.383254],
    [ 0.180957,  0.958312, -0.221119, -486.303535],
    [-0.983444,  0.178518, -0.031135, 101.640503],
    [ 0.000000,  0.000000,  0.000000,   1.000000],
])

# joint config that approaches the on button without hitting the mazzer body
joints_approach_on_button = Mat([
    [-58.194715], [-115.349600], [-110.340618],
    [-138.725704], [217.471205], [-82.621485],
])

# joint config that has the tool pressed on the on button
joints_on_button_pressed = Mat([
    [-53.578137], [-123.893478], [-95.582609],
    [-144.526326], [222.025126], [-82.049804],
])

# joint config that retreats off the on button before crossing to the off button
joints_retreat_from_on_button = Mat([
    [-55.339807], [-123.058942], [-97.068748],
    [-144.010434], [220.273563], [-82.244019],
])

# joint config that approaches the off button without hitting the mazzer body
joints_approach_off_button = Mat([
    [-55.666715], [-125.455802], [-92.788262],
    [-145.911041], [219.932085], [-82.268910],
])

# joint config that has the tool pressed on the off button
joints_off_button_pressed = Mat([
    [-54.388557], [-126.090429], [-91.641837],
    [-146.321380], [221.202451], [-82.125042],
])

# joint config that clears the mazzer body before detaching the tool
joints_pre_detach = Mat([
    [-52.821943], [-90.295722], [-121.667734],
    [-57.885532], [90.030588], [-197.342157],
])


def run_task_c(UR5, tls, RDK):
    # picks up the mazzer tool from the tool stand
    tls.mazzer_tool_attach_r_ati()

    # # moves through the staging pose and approach waypoint to avoid colliding with the mazzer body
    # UR5.MoveJ(staging_pose, True)
    # UR5.MoveJ(joints_approach_on_button, True)

    # # presses the on button, retreats and re-approaches, waits for the grind, then presses the off button
    # UR5.MoveJ(joints_on_button_pressed, True)
    # UR5.MoveL(joints_retreat_from_on_button, True)
    # UR5.MoveL(joints_approach_off_button, True)
    # sleep(15)
    # UR5.MoveL(joints_off_button_pressed, True)

    # # retreats back through the same waypoints before detaching the tool
    # UR5.MoveL(joints_approach_off_button, True)
    # UR5.MoveL(joints_retreat_from_on_button, True)
    # UR5.MoveJ(joints_pre_detach, True)
    # tls.mazzer_tool_detach_r_ati()


if __name__ == "__main__":
    RDK = Robolink()

    # --- SIMULATION SETTINGS (use when testing in RoboDK only) ---
    RDK.setRunMode(RUNMODE_SIMULATE)

    # --- REAL ROBOT SETTINGS (uncomment when running on the physical robot) ---
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # connect to the robot before running: Connect -> Connect robot, then press Connect and Get Position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM).RunCode()

    run_task_c(UR5, tls, RDK)