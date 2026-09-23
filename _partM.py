# _partM.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
import math
from _common import get_point, build_pose


def apply_rotation(R, point):
    x = R[0,0]*point[0] + R[0,1]*point[1] + R[0,2]*point[2]
    y = R[1,0]*point[0] + R[1,1]*point[1] + R[1,2]*point[2]
    z = R[2,0]*point[0] + R[2,1]*point[1] + R[2,2]*point[2]
    return (x, y, z)


def compute_cup_dispenser_pose():
    # Gets the cup dispenser's origin and a reference point, in both world and local coordinates
    origin_w = get_point(1)
    origin_l = get_point(16)
    ref_w = get_point(2)
    ref_l = get_point(17)

    translation = origin_w

    # Finds the world frame displacement between the origin and the reference point
    world_frame_displacement = tuple(ref_w[i] - origin_w[i] for i in range(3))

    # Known tilt of the fixture, found by inspection, matches the primer's cup stand example
    frame_tilt = roty(-90 * pi / 180)

    # Strips the known tilt out of the local reference point to isolate the unknown spin
    local_point = tuple(ref_l[i] - origin_l[i] for i in range(3))
    upright_point = apply_rotation(frame_tilt, local_point)

    # Solves for the remaining spin by comparing the flattened local point against the world displacement
    cross = upright_point[0] * world_frame_displacement[1] - upright_point[1] * world_frame_displacement[0]
    dot = upright_point[0] * world_frame_displacement[0] + upright_point[1] * world_frame_displacement[1]
    psi = math.atan2(cross, dot)

    # Combines the solved spin with the known tilt, with the known tilt applied first
    UR_R_CSprime = rotz(psi)
    UR_R_CS = UR_R_CSprime * frame_tilt

    cup_dispenser_pose = transl(*translation) * UR_R_CS
    return cup_dispenser_pose


def compute_task_m_shut_pose(fixture_fine_tune_mm):
    cup_dispenser_pose = compute_cup_dispenser_pose()
    cup_dispenser_pose = cup_dispenser_pose * transl(*fixture_fine_tune_mm)

    # Sets the placement point as the tab in its shut position, with the tool orientation found by inspection
    point_local = get_point(19)
    axis_mapping = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = cup_dispenser_pose * T_fixture_placement

    # Sets the mazzer tool's contact point
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    task_m_shut_pose = T_world_target * T_tool_offset.inv()

    return task_m_shut_pose


def compute_task_m_open_pose(fixture_fine_tune_mm):
    cup_dispenser_pose = compute_cup_dispenser_pose()
    cup_dispenser_pose = cup_dispenser_pose * transl(*fixture_fine_tune_mm)

    # Sets the placement point as the tab in its open position, with the tool orientation found by inspection
    point_local = get_point(18)
    axis_mapping = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = cup_dispenser_pose * T_fixture_placement

    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    return T_world_target * T_tool_offset.inv()


# Joint config that clears the cup dispenser body on approach and retreat
joints_intermediate_clear = [-136.349366, -88.043601, -120.503981, -151.742096, -46.514798, -269.491010]


def run_task_m(UR5, tls, RDK):
    # Edit these in the lab
    shut_fixture_fine_tune_mm = (0, 0, -4)
    open_fixture_fine_tune_mm = (0, 0, 0)

    shut_pose = compute_task_m_shut_pose(shut_fixture_fine_tune_mm)
    open_pose = compute_task_m_open_pose(open_fixture_fine_tune_mm)

    # Picks up the mazzer tool from the tool stand
    tls.mazzer_tool_attach_r_ati()

    # Cup drop motion, flicks the tab open then back to shut to release a cup
    UR5.MoveJ(joints_intermediate_clear, True)
    UR5.MoveJ(shut_pose, True)
    sleep(1)
    UR5.MoveJ(open_pose, True)
    sleep(1)
    UR5.MoveJ(shut_pose, True)

    # Retreats clear of the fixture before detaching the tool
    UR5.MoveJ(joints_intermediate_clear, True)
    tls.mazzer_tool_detach_r_ati()


def test_task_m(UR5, tls, RDK):
    run_task_m(UR5, tls, RDK)

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
    RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM).RunCode()

    test_task_m(UR5, tls, RDK)
