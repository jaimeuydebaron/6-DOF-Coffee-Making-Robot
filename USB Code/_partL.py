# _partLlefthand.py
# Task l, remove the rancilio tool from the puq fixture and insert it into the rancilio group head
# This sets the tool down on the mazzer scale, runs tasks m and n to dispense the cup and place it on the rancilio
# scale, since on the real cart the tool in the group head is in the way of placing the cup, then picks the tool
# back up from the scale and inserts it into the group head. All of it is in the left hand configuration
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import get_point, build_pose, solve_frame_rotation, solve_joints_near, compute_waypoints
import _partK as part_k
import _partF as part_f
import _partA as part_a
import _partM as part_m
import _partN as part_n
from _partF import compute_task_f_pose


def compute_rancilio_pose(fixture_fine_tune_mm):
    # Gets the rancilio's origin and a reference point, in both world and local coordinates
    origin_w = get_point(10)
    origin_l = get_point(35)
    ref_w = get_point(11)
    ref_l = get_point(36)

    # Solves for how far the rancilio is rotated relative to the world frame
    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    # Builds the rancilio's pose in the robot's world frame
    rancilio_pose = transl(*origin_w) * rotz(theta)

    # Applies the fixture calibration correction
    rancilio_pose = rancilio_pose * transl(*fixture_fine_tune_mm)

    return rancilio_pose


def compute_task_l_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg=0, handle_yaw_deg=0,
                        approach_offset_mm=(0, 0, 0), group_head_tilt_deg=0):
    rancilio_pose = compute_rancilio_pose(fixture_fine_tune_mm)

    # Sets the placement point as the group head gasket centre, with the tool orientation found by inspection,
    # the basket's opening facing up into the group head and the handle out along the group head's negative x axis,
    # then spun about the group head's axis by handle_yaw_deg so the handle can point out of the machine the right way
    # The group head frame (point 38) is rotated about the rancilio's positive y axis by group_head_tilt_deg, so the
    # tool is tilted with it, and the yaw is then about the group head's own tilted axis
    point_local = get_point(38)
    axis_mapping = [
        [0, 0, 1],   # Tool x axis lines up with the fixture's z axis, the basket faces up
        [0, 1, 0],   # Tool y axis lines up with the fixture's y axis
        [-1, 0, 0],  # Tool z axis lines up with the fixture's negative x axis, the handle direction
    ]
    T_fixture_orientation = build_pose(axis_mapping, (0, 0, 0))
    T_fixture_placement = transl(*point_local) * roty(group_head_tilt_deg * pi/180) * rotz(handle_yaw_deg * pi/180) * T_fixture_orientation

    # Shifts the target along the fixture's own axes by approach_offset_mm, zero for the final insertion pose
    T_world_target = rancilio_pose * transl(*approach_offset_mm) * T_fixture_placement

    # Sets the rancilio tool's contact point, the basket rim centre (point 55), and its mounting angle offset
    tool_point_local = get_point(55)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    task_l_pose = T_world_target * T_tool_offset.inv()

    # Applies the approach angle correction
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_l_pose = task_l_pose * angle_adjust

    # Tilts the tool positively about the tool's own y by the stem angle (point 63) plus extra_tilt_deg, pivoting
    # about the basket rim centre so the rim stays put, the same way as parts g and k
    tilt_deg = get_point(63)[0] + extra_tilt_deg
    task_l_pose = task_l_pose * T_tool_offset * roty(tilt_deg * pi/180) * T_tool_offset.inv()

    return task_l_pose


def compute_task_l_twist_steps(start_pose, fixture_fine_tune_mm, twist_deg, num_steps, group_head_tilt_deg=0):
    # Turns the tool about the group head's axis, through the gasket centre, in several small steps so the
    # motion follows the bayonet's rotation. The axis is the group head frame's z axis, which is tilted off the
    # vertical by group_head_tilt_deg. Positive twist_deg turns anticlockwise seen from above, which is
    # clockwise seen from below, the direction the lab frames diagram shows for locking the tool in
    rancilio_pose = compute_rancilio_pose(fixture_fine_tune_mm)
    gasket_pose = rancilio_pose * transl(*get_point(38)) * roty(group_head_tilt_deg * pi/180)

    steps = []
    for i in range(1, num_steps + 1):
        angle = twist_deg * i / num_steps
        steps.append(gasket_pose * rotz(angle * pi/180) * gasket_pose.inv() * start_pose)

    return steps


# Jogged joint waypoint that carries the tool from the puq across towards the mazzer scale, left hand
left_scale_approach_intermediate_pose_1 = [96.730000, -98.710000, 145.110000, -46.390000, 6.730000, 140.000000]

# Left hand arm configuration hint for the joint move in front of the group head, with the elbow and the wrist up
# (J5 positive), the same way round as the rest of task l and tasks m and n. It only picks which of the arm's
# solutions is used, it does not change where the tool goes
GROUP_HEAD_CONFIG_HINT = [11, -100, 144, -44, 101, 140]


def run_task_l(UR5, tls, RDK):
    """ Does not return the robot home, stays in the left hand configuration throughout, and ends empty handed,
    risen clear of the rancilio, with the tool locked in the group head and the cup on the rancilio scale """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # ---------- Setting the tool down on the mazzer scale and picking it back up, left hand ----------
    # These four MUST equal part a's values, the same as part f uses, so the tool is left and picked up at exactly
    # the position and angle it was left at, and part a's hover is the lift and lower height.

    # scale_fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the tool is set down on the scale and picked up from, along the scale's own axes.
    # z raises (+) or lowers (-) it, use it if the tool floats above or presses into the ball bearing.
    scale_fixture_fine_tune_mm = (0, 0, -4)

    # scale_approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z) when it is set down and picked up.
    # It pivots about the tool's origin, NOT the ball bearing, so changing it also moves where the tip lands.
    scale_approach_angle_adjust_deg = (0, 0, 0)

    # scale_extra_tilt_deg
    # Extra lean added on top of the 1.7 degree stem angle (point 63), about the tool's own y, pivoting about the
    # ball bearing point, so the tool rests at the same angle whichever hand is using the pose.
    scale_extra_tilt_deg = 0

    # scale_hover_mm
    # How far straight above the ball bearing the tool is lined up before it is lowered, and lifted to after
    # it is let go of or picked up. Bigger = more clearance, smaller = less.
    scale_hover_mm = 30

    # ---------- Inserting the tool into the group head, left hand ----------

    # fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the tool goes in the group head, by shifting the assumed position of the rancilio along the
    # rancilio's own axes. The tool keeps its angle and just moves as a whole, and the twist moves with it.
    #   x: shifts along the rancilio's local x
    #   y: shifts along the rancilio's local y
    #   z: raises (+) or lowers (-) the tool, use this if the basket is too low to seal or presses too hard
    fixture_fine_tune_mm = (0, 0, 0)

    # group_head_tilt_deg
    # How far the group head frame (point 38) is rotated about the positive y axis of the local rancilio frame,
    # from the lab technician. The tool is inserted and twisted along this tilted axis. Set it for the robot in use:
    #   robot 1: 1.5, robot 2: 1.6, robot 3: 1.7
    group_head_tilt_deg = 1.7

    # approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z) at the insertion pose, before the tilt.
    # It pivots about the tool's origin, NOT the basket rim centre, so changing it also moves where the basket
    # lands. Change the angle first, then re-tune fixture_fine_tune_mm to put the basket back on the gasket.
    approach_angle_adjust_deg = (0, 0, 0)

    # extra_tilt_deg
    # Extra lean added on top of the 1.7 degree stem angle (point 63), about the tool's own y, pivoting about the
    # basket rim centre so the rim stays where it is and the handle swings around it.
    # Positive = leans further, negative = reduces the tilt (-1.7 gives no tilt). Tune it until the basket rim
    # sits flat against the gasket.
    extra_tilt_deg = 0

    # handle_yaw_deg
    # Spins the tool about the group head's vertical axis, in degrees, before the twist, so it sets the angle the
    # tool goes IN at. It is measured from the handle pointing along the rancilio's -x, so 0 is -x, 180 is +x,
    # and positive turns anticlockwise seen from above.
    # The locked angle is handle_yaw_deg + twist_deg. With -45 and 45 the tool goes in 45 degrees clockwise of the
    # rancilio's -x and is twisted round to -x, which is where it locks. To move both the entry and the locked
    # angle together, change this. To move only the locked angle, change twist_deg.
    handle_yaw_deg = -45

    # approach_offset_mm = (x, y, z) in mm
    # Where the tool lines up before the final move up into the group head, as an offset from the insertion
    # pose along the rancilio's own axes. (0, 0, -20) starts 20 mm below the gasket and moves up into it.
    # It does not change the insertion pose, only the pose the move starts from.
    approach_offset_mm = (0, 0, -20)

    # twist_deg
    # How far to turn the tool once it is fully in, to lock it, anticlockwise seen from above (clockwise seen
    # from below). The lab frames diagram says the angle will likely need refining until the seal is good.
    # Bigger = turns further past the entry angle, smaller = turns less, 0 turns not at all, negative turns the
    # other way. Nudge this by a few degrees to fix a locked position that is slightly off.
    twist_deg = 45

    # num_twist_steps
    # How many points the twist is split into. More = smoother but slower, fewer = faster but jerkier.
    num_twist_steps = 9

    # front_of_group_head_mm
    # How far out in front of the rancilio (along its +x) the tool lines up, at the height of the approach pose,
    # before sliding straight in under the group head. Coming down from above instead passes through the machine.
    front_of_group_head_mm = 150

    # retreat_back_mm, retreat_up_mm
    # Once the tool is locked in and let go of, how far the robot backs straight off along the handle, out of the
    # tool's coupling, and then how far it rises before going home.
    retreat_back_mm = 60
    retreat_up_mm = 100

    # The jogged joint poses imported from parts k, f and a are NOT affected by the settings above. Jog them if
    # the arm hits the puq, the mazzer or the wdt on the way across.
    # =============================================================================

    # Computes the poses above and on the scale, for setting the tool down and picking it up again
    scale_hover_pose = compute_task_f_pose(
        scale_fixture_fine_tune_mm, scale_approach_angle_adjust_deg, scale_extra_tilt_deg, scale_hover_mm)
    scale_pose = compute_task_f_pose(
        scale_fixture_fine_tune_mm, scale_approach_angle_adjust_deg, scale_extra_tilt_deg)

    # Computes the pose lined up below the group head, the pose fully inserted, and the poses through the twist
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

    # Makes sure the poses below are read in the robot base frame with the master tool, as the assignment requires
    # The tool change routines leave the frame on the tool stand's frame until they finish, so a run that was cut
    # short, or a test that skips the attach, leaves every pose in this file measured from the wrong place
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # The tool attached to the robot, hidden and shown below as the tool moves between the robot and the fixtures
    attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)

    # Optional, builds extra waypoints from the scale pose instead of using scale_hover_mm
    # Each offset is (name, (x, y, z) in mm, (rx, ry, rz) in degrees), applied in the tool's OWN frame,
    # translation first then rotation. Each waypoint comes from scale_pose on its own, they do not chain.
    # For example, this backs the tool off 60 mm along its own z, keeping the tilt:
    waypoints = compute_waypoints(scale_pose, offsets=[
        ("backoff", (0, 0, -60), (0, 0, 0)),
    ])
    # UR5.MoveL(waypoints["backoff"], True)
    # UR5.MoveL(scale_pose, True)

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

    show_rancilio = RDK.Item("Show_Mazzer_Scale_Rancilio_Tool", ITEM_TYPE_PROGRAM)
    show_rancilio.RunCode()
    show_rancilio.WaitFinished()

    # Retreats clear of the mazzer scale, then carries straight on into task m in the left hand configuration
    UR5.MoveJ(part_a.retreat_intermediate_pose_1, True)
    UR5.MoveL(part_a.retreat_intermediate_pose_2, True)
    # UR5.MoveJ(part_a.tool_change_intermediate_pose, True)

    # ---------- Pauses task l to run tasks m and n, while the tool is sitting on the mazzer scale ----------
    # On the real cart the rancilio tool in the group head is in the way of placing the cup on the rancilio scale,
    # so the cup is dispensed and placed now, before the tool goes into the group head
    # Both run in the left hand configuration, carrying on from home_l without changing configuration, m ends with
    # the mazzer tool back on the stand and n ends empty handed at the cup tool's approach pose on the tool stand

    # part_m.run_task_m(UR5, tls, RDK)
    # part_n.run_task_n(UR5, tls, RDK)

    # ---------- Stays in the left hand configuration, and picks the tool up from the mazzer scale ----------

    # Comes back in to the mazzer scale the way the tool was set down, and lowers onto it
    UR5.MoveJ(part_a.approach_intermediate_pose_2, True)
    UR5.MoveJ(part_f.rancilio_align_intermediate_pose, True)
    UR5.MoveL(scale_pose, True)

    # Grabs the tool, hides the ghost object, and shows it attached to the robot instead, the same way part f does
    attached_tool.setVisible(True, False)

    hide_rancilio = RDK.Item("Hide_Mazzer_Scale_Rancilio_Tool", ITEM_TYPE_PROGRAM)
    hide_rancilio.RunCode()
    hide_rancilio.WaitFinished()

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

    show_rancilio_locked = RDK.Item("Show_Rancilio_Rancilio_Tool_Rotated", ITEM_TYPE_PROGRAM)
    show_rancilio_locked.RunCode()
    show_rancilio_locked.WaitFinished()

    # Backs straight off the tool along the handle and rises clear of the rancilio, ready for task o
    UR5.MoveL(retreat_back_pose, True)
    UR5.MoveL(retreat_up_pose, True)


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

    run_task_l(UR5, tls, RDK)

    # Sends the robot home
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
