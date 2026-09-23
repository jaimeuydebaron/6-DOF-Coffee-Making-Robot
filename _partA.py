# _partA.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, solve_frame_rotation


def compute_task_a_pose(fixture_fine_tune_mm, approach_angle_adjust_deg):
    # Gets the mazzer scale's origin and a reference point, in both world and local coordinates
    origin_w = get_point(6)
    origin_l = get_point(27)
    ref_w = get_point(7)
    ref_l = get_point(28)

    # Solves for how far the mazzer scale is rotated relative to the world frame
    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    # Builds the mazzer scale's pose in the robot's world frame
    mazzer_scale_pose = transl(*origin_w) * rotz(theta)

    # Applies the fixture calibration correction
    mazzer_scale_pose = mazzer_scale_pose * transl(*fixture_fine_tune_mm)

    # Sets the placement point as the ball bearing apex, with the tool orientation found by inspection
    point_local = get_point(29)
    axis_mapping = [[0, 0, 1], [0, -1, 0], [1, 0, 0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_scale_pose * T_fixture_placement

    # Sets the rancilio tool's contact point and its mounting angle offset
    tool_point_local = get_point(54)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    task_a_pose = T_world_target * T_tool_offset.inv()

    # Applies the approach angle correction
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_a_pose = task_a_pose * angle_adjust

    return task_a_pose


# Jogged waypoints that route the tool onto the mazzer scale without
# colliding with the mazzer body on the way in
approach_intermediate_pose_1 = [94.213014, -105.631792, 119.134449, -103.512723, -89.997053, -230.793487]
approach_intermediate_pose_2 = [148.740997, -103.856520, 121.433962, -17.577449, 118.713599, 139.999989]
approach_intermediate_pose_3 = [148.741161, -82.451284, 138.999911, -56.548628, 118.714037, 140.000000]


def run_task_a(UR5, tls, RDK):
    """ Callable from __main.py, this skips the intermediate return to home that test_task_a adds """

    # Edit these in the lab
    # Shifts the mazzer scale's whole pose along its own local x, y, z directions to correct calibration drift
    fixture_fine_tune_mm = (0, 0, 0)
    # Rotates the tool's approach about its own x, y, z axes once it is already oriented at the placement point
    approach_angle_adjust_deg = (0, 0, 0)

    # Computes the final pose for placing the tool on the scale
    task_a_pose = compute_task_a_pose(fixture_fine_tune_mm, approach_angle_adjust_deg)

    # Picks up the rancilio tool from the tool stand
    tls.rancilio_tool_attach_l_ati()

    # Moves in and places the tool on the scale
    UR5.MoveJ(approach_intermediate_pose_1, True)
    UR5.MoveJ(approach_intermediate_pose_2, True)
    UR5.MoveL(approach_intermediate_pose_3, True)
    UR5.MoveL(task_a_pose, True)

    # Releases the tool on the real robot
    tls.student_tool_detach()

    # Shows the ghost object left behind in the tool's place
    attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
    attached_tool.setVisible(False, False)

    show_rancilio = RDK.Item("Show_Mazzer_Scale_Rancilio_Tool", ITEM_TYPE_PROGRAM)
    show_rancilio.RunCode()
    show_rancilio.WaitFinished()

    # Retreats to an intermediate pose
    UR5.MoveL(approach_intermediate_pose_3, True)


def test_task_a(UR5, tls, RDK):
    run_task_a(UR5, tls, RDK)

    # Sends the robot home
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    test_task_a(UR5, tls, RDK)
