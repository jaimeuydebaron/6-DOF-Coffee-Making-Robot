# _partM.py
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


def apply_rotation(R, point):
    x = R[0,0]*point[0] + R[0,1]*point[1] + R[0,2]*point[2]
    y = R[1,0]*point[0] + R[1,1]*point[1] + R[1,2]*point[2]
    z = R[2,0]*point[0] + R[2,1]*point[1] + R[2,2]*point[2]
    return (x, y, z)


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


def compute_cup_dispenser_pose():
    # gets the cup dispenser origin and reference point in world and local coordinates
    origin_w = get_point(1)
    origin_l = get_point(16)
    ref_w = get_point(2)
    ref_l = get_point(17)

    translation = origin_w

    # finds the world frame displacement between origin and reference point
    world_frame_displacement = tuple(ref_w[i] - origin_w[i] for i in range(3))

    # known tilt of the fixture, found by inspection, matches the primer's cup stand example
    frame_tilt = roty(-90 * pi / 180)

    # strips the known tilt out of the local reference point to isolate the unknown spin
    local_point = tuple(ref_l[i] - origin_l[i] for i in range(3))
    upright_point = apply_rotation(frame_tilt, local_point)

    # solves for the remaining spin by comparing the flattened local point against the world displacement
    cross = upright_point[0] * world_frame_displacement[1] - upright_point[1] * world_frame_displacement[0]
    dot = upright_point[0] * world_frame_displacement[0] + upright_point[1] * world_frame_displacement[1]
    psi = math.atan2(cross, dot)

    # combines the solved spin with the known tilt, known tilt applied first
    UR_R_CSprime = rotz(psi)
    UR_R_CS = UR_R_CSprime * frame_tilt

    cup_dispenser_pose = transl(*translation) * UR_R_CS
    return cup_dispenser_pose


def compute_task_m_shut_pose(fixture_fine_tune_mm):
    cup_dispenser_pose = compute_cup_dispenser_pose()
    cup_dispenser_pose = cup_dispenser_pose * transl(*fixture_fine_tune_mm)

    # sets the placement point as the tab in its shut position and the tool's orientation there
    point_local = get_point(19)
    axis_mapping = [[1,0,0], [0,1,0], [0,0,1]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = cup_dispenser_pose * T_fixture_placement

    # sets the mazzer tool's contact point and mounting angle offset
    tool_point_local = get_point(51)
    T_tool_offset = rotz(0 * pi/180) * transl(*tool_point_local)
    task_m_shut_pose = T_world_target * T_tool_offset.inv()

    return task_m_shut_pose


def compute_task_m_open_pose(fixture_fine_tune_mm):
    cup_dispenser_pose = compute_cup_dispenser_pose()
    cup_dispenser_pose = cup_dispenser_pose * transl(*fixture_fine_tune_mm)

    # sets the placement point as the tab in its open position and the tool's orientation there
    point_local = get_point(18)
    axis_mapping = [[1,0,0], [0,1,0], [0,0,1]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = cup_dispenser_pose * T_fixture_placement

    tool_point_local = get_point(51)
    T_tool_offset = rotz(0 * pi/180) * transl(*tool_point_local)
    return T_world_target * T_tool_offset.inv()


# joint config that clears the cup dispenser body on approach and retreat
joints_intermediate_clear = Mat([
    [-136.349366],
    [-88.043601],
    [-120.503981],
    [-151.742096],
    [-46.514798],
    [-269.491010],
])


def run_task_m(UR5, tls, RDK):
    # edit these in the lab
    shut_fixture_fine_tune_mm = (0, 0, -4)
    open_fixture_fine_tune_mm = (0, 0, 0)

    shut_pose = compute_task_m_shut_pose(shut_fixture_fine_tune_mm)
    open_pose = compute_task_m_open_pose(open_fixture_fine_tune_mm)

    # waypoints built separately from the targets
    shut_waypoints = compute_waypoints(shut_pose, offsets=[])
    open_waypoints = compute_waypoints(open_pose, offsets=[])

    # picks up the mazzer tool from the tool stand
    tls.mazzer_tool_attach_r_ati()

    # # moves in to the tab and pulls it open then shut once to drop a cup
    # UR5.MoveJ(joints_intermediate_clear, True)
    # UR5.MoveJ(shut_pose, True)
    # sleep(1)
    # UR5.MoveJ(open_pose, True)
    # sleep(1)
    UR5.MoveJ(shut_pose, True)

    # retreats clear of the fixture before detaching the tool and going home
    UR5.MoveJ(joints_intermediate_clear, True)
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
    # RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM).RunCode()

    run_task_m(UR5, tls, RDK)