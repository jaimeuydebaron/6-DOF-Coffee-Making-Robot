# _partL.py
# Task l, remove the rancilio tool from the puq fixture and insert it into the rancilio group head
# This sets the tool down on the mazzer scale, runs task n to place the cup on the rancilio
# scale, since on the real cart the tool in the group head is in the way of placing the cup, then picks the tool
# back up from the scale and inserts it into the group head. All of it is in the left hand configuration
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import get_point, build_pose, solve_frame_rotation, solve_joints_near, run_visual, enable_space_pause
import _partF as part_f
import _partA as part_a
import _partN as part_n
from _partF import compute_task_f_pose


def compute_rancilio_pose(fixture_fine_tune_mm):
    # The rancilio's frame, see part a for how this works
    origin_w = get_point(10)
    origin_l = get_point(35)
    ref_w = get_point(11)
    ref_l = get_point(36)

    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    rancilio_pose = transl(*origin_w) * rotz(theta)

    rancilio_pose = rancilio_pose * transl(*fixture_fine_tune_mm)

    return rancilio_pose


def compute_task_l_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg=0, handle_yaw_deg=0,
                        approach_offset_mm=(0, 0, 0), group_head_tilt_deg=0):
    rancilio_pose = compute_rancilio_pose(fixture_fine_tune_mm)

    # The target is the group head gasket centre (point 38), basket facing up and the handle out along the
    # group head's -x. The group head frame is tilted about the rancilio's y by group_head_tilt_deg, and the
    # handle is spun about that tilted axis by handle_yaw_deg
    point_local = get_point(38)
    axis_mapping = [
        [0, 0, 1],   # Tool x axis lines up with the fixture's z axis, the basket faces up
        [0, 1, 0],   # Tool y axis lines up with the fixture's y axis
        [-1, 0, 0],  # Tool z axis lines up with the fixture's negative x axis, the handle direction
    ]
    T_fixture_orientation = build_pose(axis_mapping, (0, 0, 0))
    T_fixture_placement = transl(*point_local) * roty(group_head_tilt_deg * pi/180) * rotz(handle_yaw_deg * pi/180) * T_fixture_orientation

    # Shifted along the rancilio's own axes by approach_offset_mm, zero for the final pose
    T_world_target = rancilio_pose * transl(*approach_offset_mm) * T_fixture_placement

    # The tool's contact point is the basket rim centre (point 55)
    tool_point_local = get_point(55)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    task_l_pose = T_world_target * T_tool_offset.inv()

    # Angle adjust about the tcp
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_l_pose = task_l_pose * angle_adjust

    # Tilts about the tool's own y by the stem angle plus extra_tilt_deg, pivoting about the rim centre, see part g
    tilt_deg = get_point(63)[0] + extra_tilt_deg
    task_l_pose = task_l_pose * T_tool_offset * roty(tilt_deg * pi/180) * T_tool_offset.inv()

    return task_l_pose


def compute_task_l_twist_steps(start_pose, fixture_fine_tune_mm, twist_deg, num_steps, group_head_tilt_deg=0):
    # Turns start_pose about the group head's tilted axis through the gasket centre, in num_steps steps. Positive
    # twist_deg is anticlockwise seen from above, clockwise from below, the direction that locks the tool in
    rancilio_pose = compute_rancilio_pose(fixture_fine_tune_mm)
    gasket_pose = rancilio_pose * transl(*get_point(38)) * roty(group_head_tilt_deg * pi/180)

    steps = []
    for i in range(1, num_steps + 1):
        angle = twist_deg * i / num_steps
        steps.append(gasket_pose * rotz(angle * pi/180) * gasket_pose.inv() * start_pose)

    return steps


# Jogged joint waypoint that carries the tool from the puq across towards the mazzer scale, left hand
left_scale_approach_intermediate_pose_1 = [96.730000, -98.710000, 145.110000, -46.390000, 6.730000, -220.000000]

# Left hand hint, elbow and wrist up (J5 positive), for the joint move in front of the group head. It only picks
# which of the arm's solutions is used
GROUP_HEAD_CONFIG_HINT = [11, -100, 144, -44, 101, -220]


def run_task_l(UR5, tls, RDK):
    """ Does not return the robot home, stays in the left hand configuration throughout, and ends empty handed,
    risen clear of the rancilio, with the tool locked in the group head and the cup on the rancilio scale """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # ---------- Setting the tool down on the mazzer scale and picking it back up, left hand ----------
    # These four MUST equal part a's values, see there, so the tool is set down and picked up the same way

    scale_fixture_fine_tune_mm = (0, 0, -4)
    scale_approach_angle_adjust_deg = (0, 0, 0)
    scale_extra_tilt_deg = 0
    scale_hover_mm = 30

    # ---------- Inserting the tool into the group head, left hand ----------

    # Shifts where the tool goes in the group head, along the rancilio's own axes. z raises (+) or lowers (-) it
    fixture_fine_tune_mm = (0, 0, 0)

    # Tilt of the group head about the rancilio's y, from the lab technician. Robot 1: 1.5, robot 2: 1.6, robot 3: 1.7
    group_head_tilt_deg = 1.7

    # Rotates the tool about the tcp, so the basket moves too
    approach_angle_adjust_deg = (0, 0, 0)

    # Added to the stem angle, pivoting about the rim centre. Tune it until the rim sits flat against the gasket
    extra_tilt_deg = 0

    # The angle the tool goes in at, about the group head's axis. 0 is the handle along the rancilio's -x,
    # positive is anticlockwise seen from above. The locked angle is handle_yaw_deg + twist_deg
    handle_yaw_deg = -45

    # Where the tool lines up below the gasket before moving up into it, along the rancilio's own axes
    approach_offset_mm = (0, 0, -20)

    # How far the tool is turned to lock it, anticlockwise seen from above. Nudge it until the seal is good
    twist_deg = 45

    # Points the twist is split into
    num_twist_steps = 9

    # How far out in front of the rancilio (its +x) the tool lines up before sliding in under the group head.
    # Coming down from above instead passes through the machine
    front_of_group_head_mm = 150

    # Once the tool is let go, how far the robot backs off along the handle, then how far it rises
    retreat_back_mm = 60
    retreat_up_mm = 100

    # =============================================================================

    # The poses above and on the mazzer scale
    scale_hover_pose = compute_task_f_pose(
        scale_fixture_fine_tune_mm, scale_approach_angle_adjust_deg, scale_extra_tilt_deg, scale_hover_mm)
    scale_pose = compute_task_f_pose(
        scale_fixture_fine_tune_mm, scale_approach_angle_adjust_deg, scale_extra_tilt_deg)

    # The pose lined up below the group head, the pose fully inserted, and the poses through the twist
    task_l_approach_pose = compute_task_l_pose(
        fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg, handle_yaw_deg, approach_offset_mm,
        group_head_tilt_deg)
    task_l_pose = compute_task_l_pose(
        fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg, handle_yaw_deg,
        group_head_tilt_deg=group_head_tilt_deg)
    twist_steps = compute_task_l_twist_steps(
        task_l_pose, fixture_fine_tune_mm, twist_deg, num_twist_steps, group_head_tilt_deg)

    # The pose lined up in front of the rancilio, level with the approach pose, along the rancilio's +x
    rancilio_pose = compute_rancilio_pose(fixture_fine_tune_mm)
    out_front = [rancilio_pose[i, 0] * front_of_group_head_mm for i in range(2)]
    front_of_group_head_pose = transl(out_front[0], out_front[1], 0) * task_l_approach_pose

    # The poses backed off the locked tool, along the handle, then raised
    retreat_back_pose = twist_steps[-1] * transl(0, 0, -retreat_back_mm)
    retreat_up_pose = transl(0, 0, retreat_up_mm) * retreat_back_pose

    # Reads the poses in the robot base frame with the master tool. The tool change routines leave the frame on
    # the tool stand's until they finish, so a run cut short would otherwise measure every pose from there
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # The tool attached to the robot, hidden and shown below as the tool moves between the robot and the fixtures
    attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)

    # ---------- Left hand, takes the tool out of the puq and sets it down on the mazzer scale ----------

    # Comes back the way part k went in, then across to the mazzer scale the way part f goes in
    UR5.MoveJ(left_scale_approach_intermediate_pose_1, True)
    UR5.MoveJ(part_a.approach_intermediate_pose_3, True)

    # Lowers the tool onto the ball bearing, the same way part a does
    UR5.MoveL(scale_hover_pose, True)
    UR5.MoveL(scale_pose, True)

    # Releases the tool, and shows the ghost object left behind in the tool's place, the same way part a does
    tls.student_tool_detach()
    attached_tool.setVisible(False, False)
    run_visual(RDK, "Show_Mazzer_Scale_Rancilio_Tool")

    # Retreats clear of the mazzer scale, then carries straight on into task n in the left hand configuration
    UR5.MoveJ(part_a.retreat_intermediate_pose_1, True)
    UR5.MoveL(part_a.retreat_intermediate_pose_2, True)

    # ---------- Runs task n while the tool sits on the mazzer scale ----------
    # The tool in the group head would be in the way of placing the cup, so the cup is placed first. Task m has
    # already dispensed it, inside task e

    part_n.run_task_n(UR5, tls, RDK)

    # ---------- Stays in the left hand configuration, and picks the tool up from the mazzer scale ----------

    # Comes back in to the mazzer scale the way the tool was set down, and lowers onto it
    UR5.MoveJ(part_a.approach_intermediate_pose_2, True)
    UR5.MoveJ(part_f.rancilio_align_intermediate_pose, True)
    UR5.MoveL(scale_pose, True)

    # Grabs the tool, hides the ghost object, and shows it attached to the robot instead, the same way part f does
    attached_tool.setVisible(True, False)
    run_visual(RDK, "Hide_Mazzer_Scale_Rancilio_Tool")

    tls.student_tool_attach()

    # Lifts straight up, now holding the tool, then backs out the same way it came in
    UR5.MoveL(scale_hover_pose, True)
    UR5.MoveJ(part_f.retreat_intermediate_pose_1, True)
    UR5.MoveJ(part_a.approach_intermediate_pose_2, True)

    # ---------- Left hand, carries the tool across and inserts it into the group head ----------

    # Carries the tool across to line up in front of the rancilio, then slides straight in under the group head
    UR5.MoveJ(solve_joints_near(UR5, front_of_group_head_pose, GROUP_HEAD_CONFIG_HINT), True)
    UR5.MoveL(task_l_approach_pose, True)

    # Moves up until the basket meets the gasket
    UR5.MoveJ(task_l_pose, True)

    # Twists the tool to lock it into the group head
    for step_pose in twist_steps:
        UR5.MoveL(step_pose, True)

    # Releases the tool, leaving it locked in the group head, and shows it there in place of the attached one
    tls.student_tool_detach()
    attached_tool.setVisible(False, False)
    run_visual(RDK, "Show_Rancilio_Rancilio_Tool_Rotated")

    # Backs straight off the tool along the handle and rises clear of the rancilio, ready for task o
    UR5.MoveL(retreat_back_pose, True)
    UR5.MoveL(retreat_up_pose, True)


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

    # Picks up the rancilio tool, in the full run it is already held, carried over from task k
    tls.rancilio_tool_attach_l_ati()

    run_task_l(UR5, tls, RDK)

    # Sends the robot home, in the full run task o carries straight on from above the rancilio
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
