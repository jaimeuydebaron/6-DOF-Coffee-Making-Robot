# _partK.py
# Task k, place the rancilio tool into the puq fixture and wait 2 seconds while the machine tamps
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, solve_frame_rotation, compute_waypoints


def compute_puq_pose(fixture_fine_tune_mm):
    # Gets the puq's origin and a reference point, in both world and local coordinates
    origin_w = get_point(8)
    origin_l = get_point(32)
    ref_w = get_point(9)
    ref_l = get_point(33)

    # Solves for how far the puq is rotated relative to the world frame
    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    # Builds the puq's pose in the robot's world frame
    puq_pose = transl(*origin_w) * rotz(theta)

    # Applies the fixture calibration correction
    puq_pose = puq_pose * transl(*fixture_fine_tune_mm)

    return puq_pose


def compute_task_k_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg=0, handle_yaw_deg=0,
                        approach_offset_mm=(0, 0, 0)):
    puq_pose = compute_puq_pose(fixture_fine_tune_mm)

    # Sets the placement point as the upper clamp centre, with the tool orientation found by inspection,
    # the basket's opening facing up into the clamp and the handle out along the fixture's negative x axis,
    # then spun about the vertical by handle_yaw_deg so the handle can point out of the puq the right way
    point_local = get_point(34)
    axis_mapping = [
        [0, 0, 1],   # Tool x axis lines up with the fixture's z axis, the basket faces up
        [0, 1, 0],   # Tool y axis lines up with the fixture's y axis
        [-1, 0, 0],  # Tool z axis lines up with the fixture's negative x axis, the handle direction
    ]
    T_fixture_orientation = build_pose(axis_mapping, (0, 0, 0))
    T_fixture_placement = transl(*point_local) * rotz(handle_yaw_deg * pi/180) * T_fixture_orientation

    # Shifts the target along the fixture's own axes by approach_offset_mm, zero for the final placement pose
    T_world_target = puq_pose * transl(*approach_offset_mm) * T_fixture_placement

    # Sets the rancilio tool's contact point, the basket rim centre (point 55), and its mounting angle offset
    tool_point_local = get_point(55)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    task_k_pose = T_world_target * T_tool_offset.inv()

    # Applies the approach angle correction
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_k_pose = task_k_pose * angle_adjust

    # Tilts the tool positively about the TOOL's own y by the stem angle (point 63) plus extra_tilt_deg, pivoting
    # about the basket rim centre so the rim stays put, the same way as part g. The stem angle is the basket
    # rim's lean relative to the handle, so this is what should bring the rim level in the puq
    tilt_deg = get_point(63)[0] + extra_tilt_deg
    task_k_pose = task_k_pose * T_tool_offset * roty(tilt_deg * pi/180) * T_tool_offset.inv()

    return task_k_pose


# Jogged joint waypoint that carries the held rancilio tool from the wdt area across to the puq without
# hitting either fixture. Jog this in the lab, leave as None to go straight to the approach pose
approach_intermediate_pose_1 = [126.730000, -98.710000, 145.110000, -46.390000, 16.730000, 140.000000]
approach_intermediate_pose_2 = [116.730000, -98.710000, 145.110000, -46.390000, 6.730000, 140.000000]
approach_intermediate_pose_3 = [96.730000, -93.110000, 147.360000, -54.240000, -23.270000, 140.000000]
pre_insert_intermediate_pose = [103.001510, -88.316925, -214.563363, -52.646337, -37.154132, 136.432017]


def run_task_k(UR5, tls, RDK):
    """ Does not return the robot home, and leaves the tool held in the puq clamp for task l to remove """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the tool goes in the puq, by shifting the assumed position of the puq along the puq's own axes.
    # The tool keeps its angle and just moves as a whole.
    #   x: shifts along the puq's local x
    #   y: shifts along the puq's local y
    #   z: raises (+) or lowers (-) the tool, use this if the basket is too high or too low in the clamp
    fixture_fine_tune_mm = (-2, 0, 0)

    # approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z) at the placement pose, before the tilt.
    # It pivots about the tool's origin, NOT the basket rim centre, so changing it also moves where the basket
    # lands. Change the angle first, then re-tune fixture_fine_tune_mm to put the basket back in the clamp.
    approach_angle_adjust_deg = (0, 0, 0)

    # extra_tilt_deg
    # Extra lean added on top of the 1.7 degree stem angle (point 63), about the tool's own y, pivoting about the
    # basket rim centre so the rim stays where it is and the handle swings around it.
    # Positive = leans further, negative = reduces the tilt (-1.7 gives no tilt). The point of this is to get the
    # basket rim level under the puq, so tune it until the tamper meets the grounds flat.
    extra_tilt_deg = 1

    # handle_yaw_deg
    # Spins the tool about the puq's vertical axis through the clamp, in degrees. The direction the handle points
    # out of the puq was a guess, so this is the knob to fix it. 0 puts the handle out along the puq's -x,
    # 180 flips it to +x, 90 and -90 put it along the puq's y.
    handle_yaw_deg = 0

    # approach_offset_mm = (x, y, z) in mm
    # Where the tool lines up before the final straight move in, as an offset from the placement pose along the
    # puq's own axes. The default (0, 0, -50) starts 50 mm below the clamp and moves straight up into it. If the
    # basket has to slide in sideways along the clamp arm instead, use x or y and set z to 0.
    # It does not change the final placement pose, only the pose the move starts from.
    approach_offset_mm = (0, 0, -50)

    # tamp_seconds
    # How long to hold still in the clamp while the puq tamps, the assignment asks for 2 seconds.
    tamp_seconds = 2

    # The joint pose approach_intermediate_pose is NOT affected by the settings above, jog it if the arm hits the
    # wdt or the puq on the way across.
    # =============================================================================

    # Computes the pose lined up outside the clamp, and the final pose with the basket in the clamp
    task_k_approach_pose = compute_task_k_pose(
        fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg, handle_yaw_deg, approach_offset_mm)
    task_k_pose = compute_task_k_pose(
        fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg, handle_yaw_deg)

    # Optional, builds extra waypoints from the final tilted pose instead of using align_hover_mm
    # Each offset is (name, (x, y, z) in mm, (rx, ry, rz) in degrees), applied in the tool's OWN frame,
    # translation first then rotation. Each waypoint comes from task_a_pose on its own, they do not chain.
    # For example, this backs the tool off 40 mm along its own z, keeping the tilt:
    waypoints = compute_waypoints(task_k_pose, offsets=[
        ("backoff", (0, 0, -70), (0, 0, 0)),
    ])
    # UR5.MoveL(waypoints["backoff"], True)
    # UR5.MoveL(task_a_pose, True)

    # The rancilio tool is already held, carried over from task j
    # Moves in and places the rancilio tool into the puq clamp
    UR5.MoveJ(approach_intermediate_pose_1, True)
    UR5.MoveJ(approach_intermediate_pose_2, True)
    UR5.MoveJ(approach_intermediate_pose_3, True)

    UR5.MoveL(pre_insert_intermediate_pose, True)
    UR5.MoveL(task_k_pose, True)

    # # Holds the tool still while the puq tamps the grounds
    sleep(tamp_seconds)

    UR5.MoveL(pre_insert_intermediate_pose, True)

    # Sends the robot home
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    # RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    run_task_k(UR5, tls, RDK)


