# _partK.py
# Task k, place the rancilio tool into the puq fixture and wait 2 seconds while the machine tamps
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, solve_frame_rotation, enable_space_pause


def compute_puq_pose(fixture_fine_tune_mm):
    # The puq's frame, see part a for how this works
    origin_w = get_point(8)
    origin_l = get_point(32)
    ref_w = get_point(9)
    ref_l = get_point(33)

    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    puq_pose = transl(*origin_w) * rotz(theta)

    puq_pose = puq_pose * transl(*fixture_fine_tune_mm)

    return puq_pose


def compute_task_k_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg=0, handle_yaw_deg=0,
                        approach_offset_mm=(0, 0, 0)):
    puq_pose = compute_puq_pose(fixture_fine_tune_mm)

    # The target is the upper clamp centre (point 34), basket facing up into the clamp and the handle out along
    # the puq's -x, then spun about the vertical by handle_yaw_deg
    point_local = get_point(34)
    axis_mapping = [
        [0, 0, 1],   # Tool x axis lines up with the fixture's z axis, the basket faces up
        [0, 1, 0],   # Tool y axis lines up with the fixture's y axis
        [-1, 0, 0],  # Tool z axis lines up with the fixture's negative x axis, the handle direction
    ]
    T_fixture_orientation = build_pose(axis_mapping, (0, 0, 0))
    T_fixture_placement = transl(*point_local) * rotz(handle_yaw_deg * pi/180) * T_fixture_orientation

    # Shifted along the puq's own axes by approach_offset_mm, zero for the final pose
    T_world_target = puq_pose * transl(*approach_offset_mm) * T_fixture_placement

    # The tool's contact point is the basket rim centre (point 55)
    tool_point_local = get_point(55)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    task_k_pose = T_world_target * T_tool_offset.inv()

    # Angle adjust about the tcp
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_k_pose = task_k_pose * angle_adjust

    # Tilts about the tool's own y by the stem angle plus extra_tilt_deg, pivoting about the rim centre, the same
    # as part g. This is what brings the rim level in the puq
    tilt_deg = get_point(63)[0] + extra_tilt_deg
    task_k_pose = task_k_pose * T_tool_offset * roty(tilt_deg * pi/180) * T_tool_offset.inv()

    return task_k_pose


# Jogged joint waypoints that carry the held rancilio tool from the wdt across to the puq, clear of both
approach_intermediate_pose_3 = [96.730000, -93.110000, 147.360000, -54.240000, -23.270000, -220.00]
pre_insert_intermediate_pose = [103.001510, -88.316925, 145.436637, -52.646337, -37.154132, -223.567983]


def run_task_k(UR5, tls, RDK):
    """ Does not return the robot home, ends holding the rancilio tool, backed out of the puq """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # Shifts where the basket goes in the puq, along the puq's own axes. z raises (+) or lowers (-) it in the clamp
    fixture_fine_tune_mm = (-2, 0, 0)

    # Rotates the tool about the tcp, so the basket moves too, re-tune the fine tune after changing it
    approach_angle_adjust_deg = (0, 0, 0)

    # Added to the stem angle, pivoting about the rim centre. Tune it until the tamper meets the grounds flat
    extra_tilt_deg = 1

    # Spins the handle about the puq's vertical axis, 0 is along the puq's -x, 180 is +x
    handle_yaw_deg = 0

    # How long to hold still while the puq tamps, the assignment asks for 2 seconds
    tamp_seconds = 2

    # =============================================================================

    # The final pose with the basket in the clamp
    task_k_pose = compute_task_k_pose(
        fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg, handle_yaw_deg)

    # The rancilio tool is already held, carried over from task j
    # Moves in and places the rancilio tool into the puq clamp
    UR5.MoveJ(approach_intermediate_pose_3, True)

    UR5.MoveL(pre_insert_intermediate_pose, True)
    UR5.MoveL(task_k_pose, True)

    # Holds the tool still while the puq tamps the grounds
    sleep(tamp_seconds)

    UR5.MoveL(pre_insert_intermediate_pose, True)


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

    RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # Picks up the rancilio tool, in the full run it is already held, carried over from task j
    tls.rancilio_tool_attach_l_ati()

    run_task_k(UR5, tls, RDK)

    # # Returns the rancilio tool and sends the robot home, in the full run task l carries on still holding it
    # tls.rancilio_tool_detach_l_ati()
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
