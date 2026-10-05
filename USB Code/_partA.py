# _partA.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, solve_frame_rotation, rotate_about_tip, compute_waypoints

# Needs to come in at a slight angle, go in slightly higher, then drop it in

def tilt_tool_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm=0, extra_tilt_deg=0):
    # Gets the untilted pose, then tilts it positively about the tool's own y by the stem angle (point 63)
    # plus extra_tilt_deg, pivoting about point 54 so the tip stays on the ball bearing
    task_a_pose = compute_task_a_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm)
    tilt_deg = get_point(63)[0] + extra_tilt_deg
    return rotate_about_tip(task_a_pose, get_point(54), roty(tilt_deg * pi/180))


def compute_task_a_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm=0):
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
    # Lifts the target straight up along the scale's z by hover_mm, so the tool can line up above the ball bearing first
    T_world_target = mazzer_scale_pose * transl(0, 0, hover_mm) * T_fixture_placement

    # Sets the rancilio tool's contact point (point 54, the adapter's indent) and its mounting angle offset,
    # so that point 54 lands on point 29 when hover_mm is 0
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
approach_intermediate_pose_3 = [148.740997, -87.311857, 137.310248, -49.998397, 118.713599, 139.999989]

retreat_intermediate_pose_1 = [147.869438, -79.274625, 137.134086, -59.110450, 119.138938, 139.403203]
retreat_intermediate_pose_2 = [147.869587, -102.441040, 117.125449, -15.935730, 119.139464, 139.402718]
tool_change_intermediate_pose = [118.506782, -100.274943, 101.666651, -91.307901, -89.525705, -206.644186]

def run_task_a(UR5, tls, RDK):
    """ Does not return the robot home, __main.py chains straight into the next task """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the tool lands, by shifting the assumed position of the mazzer scale along the scale's own axes.
    # The whole tool moves with it (the tilt does not change), and the hover pose moves by the same amount.
    #   x: slides the landing point along the scale's local x
    #   y: slides the landing point along the scale's local y
    #   z: raises (+) or lowers (-) the landing point, use this if the tool is floating above or pressing into the bearing
    fixture_fine_tune_mm = (0, 0, -4)

    # approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z), applied before the extra tilt.
    # It pivots about the tool's origin rather than the ball bearing, so unlike extra_tilt_deg it also shifts
    # where the tip lands. Use extra_tilt_deg to lean the tool, and this for small corrections about x or z.
    approach_angle_adjust_deg = (0, 0, 0)

    # align_hover_mm
    # How far straight above the ball bearing (along the scale's z) the tool lines up before dropping down.
    # Bigger = more clearance at the align pose and a longer straight drop, smaller = less clearance.
    # It does not change the final resting pose, only the align pose.
    align_hover_mm = 30

    # extra_tilt_deg
    # Extra lean added on top of the 1.7 degree stem angle (point 63), about the tool's own y, pivoting about the ball
    # bearing indent (point 54) so the tip stays where it is and only the tool's angle changes.
    # Positive = leans further in the tilt direction, negative = reduces the tilt (-1.7 gives no tilt).
    # Applied to both the align pose and the final pose. Part F must use the same total tilt to pick it back up.
    extra_tilt_deg = 0

    # =============================================================================

    # Computes the pose lined up above the ball bearing, and the final pose for placing the tool on the scale
    task_a_align_pose = tilt_tool_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, align_hover_mm, extra_tilt_deg)
    task_a_pose = tilt_tool_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, 0, extra_tilt_deg)

    # Untilted versions for comparison
    task_a_align_pose_untilted = compute_task_a_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, align_hover_mm)
    task_a_pose_untilted = compute_task_a_pose(fixture_fine_tune_mm, approach_angle_adjust_deg)

    # Optional, builds extra waypoints from the final tilted pose instead of using align_hover_mm
    # Each offset is (name, (x, y, z) in mm, (rx, ry, rz) in degrees), applied in the tool's OWN frame,
    # translation first then rotation. Each waypoint comes from task_a_pose on its own, they do not chain.
    # For example, this backs the tool off 40 mm along its own z, keeping the tilt:
    waypoints = compute_waypoints(task_a_pose, offsets=[
        ("backoff", (0, 0, -80), (0, 0, 0)),
    ])
    # UR5.MoveL(waypoints["backoff"], True)
    # UR5.MoveL(task_a_pose, True)

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

    show_rancilio = RDK.Item("Show_Mazzer_Scale_Rancilio_Tool", ITEM_TYPE_PROGRAM)
    show_rancilio.RunCode()
    show_rancilio.WaitFinished()

    # Retreats to an intermediate pose
    UR5.MoveL(retreat_intermediate_pose_1, True)
    UR5.MoveL(retreat_intermediate_pose_2, True)
    UR5.MoveJ(tool_change_intermediate_pose, True)

    # Sends the robot home, comment out if doing full run
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

    run_task_a(UR5, tls, RDK)