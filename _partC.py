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


def compute_task_c_on_pose(fixture_fine_tune_mm, approach_angle_adjust_deg):
    mazzer_pose = compute_mazzer_pose()
    mazzer_pose = mazzer_pose * transl(*fixture_fine_tune_mm)

    # sets the placement point as the on button and the tool's orientation there
    point_local = get_point(26)
    axis_mapping = [[0,1,0], [0,0,-1], [-1,0,0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    # sets the mazzer tool's contact point and mounting angle offset
    tool_point_local = get_point(51)
    T_tool_offset = rotz(0 * pi/180) * transl(*tool_point_local)
    task_c_on_pose = T_world_target * T_tool_offset.inv()

    # applies the approach angle correction
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_c_on_pose = task_c_on_pose * angle_adjust

    return task_c_on_pose


def compute_task_c_off_pose(fixture_fine_tune_mm, approach_angle_adjust_deg):
    mazzer_pose = compute_mazzer_pose()
    mazzer_pose = mazzer_pose * transl(*fixture_fine_tune_mm)

    # sets the placement point as the off button and the tool's orientation there
    point_local = get_point(25)
    axis_mapping = [[0,1,0], [0,0,-1], [-1,0,0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    tool_point_local = get_point(51)
    T_tool_offset = rotz(0 * pi/180) * transl(*tool_point_local)
    task_c_off_pose = T_world_target * T_tool_offset.inv()

    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_c_off_pose = task_c_off_pose * angle_adjust

    return task_c_off_pose


# jogged pose that clears the mazzer body on the way in from home
staging_pose = Mat([
    [ 0.576111,  0.817367, -0.002682, 172.306970],
    [ 0.817338, -0.576114, -0.007157, -409.750946],
    [-0.007395,  0.001931, -0.999971, 176.463932],
    [ 0.000000,  0.000000,  0.000000,   1.000000],
])

# joint angles pose that ensures that we enter on approach at the right angle
on_approach_joint_angles = Mat([
    [99.408373],
    [-61.514148],
    [105.709453],
    [-40.390944],
    [22.650255],
    [105.236545],
])

# joint angles pose that ensures that we enter off approach at the right angle
off_approach_joint_angles = Mat([
    [99.693733],
    [-59.564093],
    [102.529120],
    [-39.203535],
    [22.933805],
    [105.284723],
])

def run_task_c(UR5, tls, RDK):
    # edit these in the lab

 

    on_fixture_fine_tune_mm = (-32, -20, 50)
    on_approach_angle_adjust_deg = (-40, 15, 0)

    off_fixture_fine_tune_mm = (-32, -20, 50)
    off_approach_angle_adjust_deg = (-40, 15, 0)

    tls.mazzer_tool_attach_l_ati()

    on_pose = compute_task_c_on_pose(on_fixture_fine_tune_mm, on_approach_angle_adjust_deg)
    off_pose = compute_task_c_off_pose(off_fixture_fine_tune_mm, off_approach_angle_adjust_deg)

    # waypoints are built separately from the targets, unaffected by the fine-tune inputs above
    on_waypoints = compute_waypoints(on_pose, offsets=[("approach", (0, 0, -50), (0, 0, 0))]) # to determine pose in space, not used for movement
    off_waypoints = compute_waypoints(off_pose, offsets=[("approach", (0, 0, -50), (0, 0, 0))])

    UR5.MoveJ(staging_pose, True)
    UR5.MoveJ(on_approach_joint_angles, True)
    # UR5.MoveJ(on_pose, True)
    # UR5.MoveL(on_approach_joint_angles, True)

    sleep(3)
    # # sleep(15)

    # UR5.MoveL(off_approach_joint_angles)
    # UR5.MoveL(off_pose, True)
    # UR5.MoveL(off_waypoints["approach"], True) # this exists to tune the joint angle to avoid collision, you need to first tune off pose, then apply this MoveL, then change the config and take that joint angle and paste, the controller might try another config if you dont force it, thats why you dont use this
    # UR5.MoveL(off_approach_joint_angles)         

    
    UR5.MoveJ(staging_pose, True)
    tls.mazzer_tool_detach_l_ati()
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)

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

    run_task_c(UR5, tls, RDK)