# _partA.py
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import get_point, build_pose, solve_frame_rotation, rotate_about_tip, run_visual, enable_space_pause

# Needs to come in at a slight angle, go in slightly higher, then drop it in

# How a task pose is calculated. Every part follows these steps, so they are explained here once and the other
# parts only note what differs
#   1. Fixture frame. Each fixture has an origin and a reference point surveyed in both world and local
#      coordinates (__points.py). solve_frame_rotation compares the two to get the fixture's rotation about the
#      world z, so fixture_pose = transl(origin_w) * rotz(theta)
#   2. Fine tune. fixture_fine_tune_mm shifts that frame along the fixture's own axes, which moves every pose
#      built on it
#   3. Target. T_fixture_placement = build_pose(axis_mapping, point_local) is a frame at the contact point on the
#      fixture. axis_mapping lists the tool's x, y and z axes written in the fixture's axes, found by inspection.
#      T_world_target = fixture_pose * T_fixture_placement
#   4. Tool. T_tool_offset goes from the tcp (the master tool) to the tool's own contact point. The rancilio and
#      cup tools are mounted 50 degrees round from the master tool, hence the rotz(-50), the mazzer tool is not.
#      pose = T_world_target * T_tool_offset.inv() is the tcp pose that puts the contact point on the target
#   5. Angle adjust. approach_angle_adjust_deg rotates the tool about x, then y, then z. Applied directly it
#      pivots about the tcp, so the contact point moves too and the fine tune needs redoing. Applied through
#      rotate_about_tip it pivots about the contact point, which stays put (see part b)
#   6. Tilt, rancilio tool only. The basket rim leans by the stem angle (point 63, 1.7 degrees) relative to the
#      handle, so the tool is tilted about y by that plus extra_tilt_deg, pivoting about the contact point
# Hover, approach and backoff poses are the same calculation with an offset added, or come from compute_waypoints
# Jogged joint poses are fixed joint angles and do not move with any setting, re-jog them if a fixture moves

def tilt_tool_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm=0, extra_tilt_deg=0):
    # Step 6, the untilted pose tilted by the stem angle plus extra_tilt_deg, pivoting at point 54
    task_a_pose = compute_task_a_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm)
    tilt_deg = get_point(63)[0] + extra_tilt_deg
    return rotate_about_tip(task_a_pose, get_point(54), roty(tilt_deg * pi/180))


def compute_task_a_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm=0):
    # Step 1, the mazzer scale's origin and reference point, in world and local coordinates
    origin_w = get_point(6)
    origin_l = get_point(27)
    ref_w = get_point(7)
    ref_l = get_point(28)

    # Its rotation about the world z
    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    # The mazzer scale's pose in the world frame
    mazzer_scale_pose = transl(*origin_w) * rotz(theta)

    # Step 2, fine tune
    mazzer_scale_pose = mazzer_scale_pose * transl(*fixture_fine_tune_mm)

    # Step 3, the target is the ball bearing apex (point 29), raised along the scale's z by hover_mm
    point_local = get_point(29)
    axis_mapping = [[0, 0, 1], [0, -1, 0], [1, 0, 0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_scale_pose * transl(0, 0, hover_mm) * T_fixture_placement

    # Step 4, puts the tool's ball bearing indent (point 54) on the target
    tool_point_local = get_point(54)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    task_a_pose = T_world_target * T_tool_offset.inv()

    # Step 5, angle adjust about the tcp
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_a_pose = task_a_pose * angle_adjust

    return task_a_pose


# Jogged waypoints that route the tool onto the mazzer scale without
# colliding with the mazzer body on the way in
approach_intermediate_pose_1 = [94.213014, -105.631792, 119.134449, -103.512723, -89.997053, -230.793487]
approach_intermediate_pose_2 = [148.740000, -103.860000, 121.430000, -17.580000, 118.710000, -220.000000]
approach_intermediate_pose_3 = [148.740997, -87.311857, 137.310248, -49.998397, 118.713599, -220.000000]

retreat_intermediate_pose_1 = [147.869438, -79.274625, 137.134086, -59.110450, 119.138938, -220.596797]
retreat_intermediate_pose_2 = [147.869587, -102.441040, 117.125449, -15.935730, 119.139464,-220.597282]
tool_change_intermediate_pose = [118.506782, -100.274943, 101.666651, -91.307901, -89.525705, -206.644186]

def run_task_a(UR5, tls, RDK):
    """ Does not return the robot home, __main.py chains straight into the next task """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # Shifts where the tool lands, along the scale's own axes (x, y, z in mm). z raises (+) or lowers (-) it
    fixture_fine_tune_mm = (0, 0, -4)

    # Rotates the tool about the tcp (rx, ry, rz in degrees), so it also shifts where the tip lands
    approach_angle_adjust_deg = (0, 0, 0)

    # Height above the ball bearing, along the scale's z, where the tool lines up before dropping straight down
    align_hover_mm = 30

    # Added to the stem angle, -1.7 gives no tilt. Part f must use the same tilt to pick the tool back up
    extra_tilt_deg = 0

    # =============================================================================

    # The pose lined up above the ball bearing, and the final pose on it
    task_a_align_pose = tilt_tool_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, align_hover_mm, extra_tilt_deg)
    task_a_pose = tilt_tool_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, 0, extra_tilt_deg)

    # Starts the robot at home
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)

    # Picks up the rancilio tool from the tool stand
    tls.rancilio_tool_attach_l_ati()

    # Moves in and places the tool on the scale
    UR5.MoveJ(approach_intermediate_pose_1, True)
    UR5.MoveJ(approach_intermediate_pose_2, True)
    UR5.MoveL(approach_intermediate_pose_3, True)
    UR5.MoveL(task_a_align_pose, True)
    UR5.MoveL(task_a_pose, True)

    # Releases the tool on the real robot
    tls.student_tool_detach()

    # Shows the ghost object left behind in the tool's place
    attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
    attached_tool.setVisible(False, False)
    run_visual(RDK, "Show_Mazzer_Scale_Rancilio_Tool")

    # Retreats to an intermediate pose
    UR5.MoveL(retreat_intermediate_pose_1, True)
    UR5.MoveL(retreat_intermediate_pose_2, True)
    UR5.MoveJ(tool_change_intermediate_pose, True)


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

    run_task_a(UR5, tls, RDK)

    # Sends the robot home, in the full run task b carries straight on from here
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
