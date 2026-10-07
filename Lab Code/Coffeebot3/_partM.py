# _partM.py
from robodk.robolink import *
from robodk.robomath import *
import tools
import math
from _common import get_point, build_pose, run_visual, enable_space_pause


def apply_rotation(R, point):
    x = R[0,0]*point[0] + R[0,1]*point[1] + R[0,2]*point[2]
    y = R[1,0]*point[0] + R[1,1]*point[1] + R[1,2]*point[2]
    z = R[2,0]*point[0] + R[2,1]*point[1] + R[2,2]*point[2]
    return (x, y, z)


def compute_cup_dispenser_pose():
    # The cup dispenser's frame. Unlike the other fixtures it is tilted, so the known tilt is taken out first and
    # the remaining spin about the world z is solved the same way as solve_frame_rotation
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
    # Same steps as part a, on the cup dispenser's frame, with no angle adjust
    cup_dispenser_pose = compute_cup_dispenser_pose()
    cup_dispenser_pose = cup_dispenser_pose * transl(*fixture_fine_tune_mm)

    # The target is the tab in its shut position (point 19)
    point_local = get_point(19)
    axis_mapping = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = cup_dispenser_pose * T_fixture_placement

    # The tool's contact point is its tip (point 51)
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    task_m_shut_pose = T_world_target * T_tool_offset.inv()

    return task_m_shut_pose


def compute_task_m_open_pose(fixture_fine_tune_mm):
    cup_dispenser_pose = compute_cup_dispenser_pose()
    cup_dispenser_pose = cup_dispenser_pose * transl(*fixture_fine_tune_mm)

    # Same as the shut pose, at the tab's open position (point 18)
    point_local = get_point(18)
    axis_mapping = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = cup_dispenser_pose * T_fixture_placement

    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    return T_world_target * T_tool_offset.inv()


# Joint config that clears the cup dispenser body on approach and retreat
staging_intermediate_pose = [118.506308, -94.689040, 121.189746, -116.417060, -89.525329, -206.644986]
approach_intermediate_pose_1 = [6.932440, -91.974178, 120.520913, -28.335085, 96.767328, 90.334583]
approach_intermediate_pose_2 = [31.860000, -108.680000, 131.790000, -112.980000, -90.000000, -213.290000]


def run_task_m(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================
    # The tool flicks the tab from shut to open and back to shut, which releases a cup. Each position has its own
    # fine tune, along the cup dispenser's own axes

    # Where the tool sits with the tab shut, the first and the last move
    shut_fixture_fine_tune_mm = (0, 0, -4)

    # Where the tool sits with the tab open, sets how far the tab is pushed
    open_fixture_fine_tune_mm = (0, 0, 0)
    # =============================================================================

    shut_pose = compute_task_m_shut_pose(shut_fixture_fine_tune_mm)
    open_pose = compute_task_m_open_pose(open_fixture_fine_tune_mm)

    # The mazzer tool is already held, carried over from task e

    UR5.MoveJ(staging_intermediate_pose, True)

    # Cup drop motion, flicks the tab open then back to shut to release a cup
    UR5.MoveJ(approach_intermediate_pose_2, True)
    UR5.MoveJ(approach_intermediate_pose_1, True)
    UR5.MoveJ(shut_pose, True)
    UR5.MoveJ(open_pose, True)
    run_visual(RDK, "Show_Cup_Dispenser_Open")
    UR5.MoveJ(shut_pose, True)
    run_visual(RDK, "Show_Cup_Dispenser_Shut")
    run_visual(RDK, "Show_Cup_Dispenser_Cup")

    # Retreats clear of the fixture, task e returns the mazzer tool to the stand afterward
    UR5.MoveJ(approach_intermediate_pose_1, True)
    UR5.MoveJ(approach_intermediate_pose_2, True)


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)

    # Space in the terminal pauses and plays the run, for testing this task on its own
    enable_space_pause(UR5, tls)

    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # Shows the rancilio tool sitting on the mazzer scale and takes it off the tool stand, as task a leaves it.
    # Simulation only
    run_visual(RDK, "Show_Mazzer_Scale_Rancilio_Tool")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    # Picks up the mazzer tool, in the full run it is already held, carried over from task e
    tls.mazzer_tool_attach_l_ati()

    run_task_m(UR5, tls, RDK)

    # Returns the mazzer tool to the tool stand, in the full run task e does this
    tls.mazzer_tool_detach_l_ati()

    # Sends the robot home
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
