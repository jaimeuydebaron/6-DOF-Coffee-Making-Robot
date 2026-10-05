# _partS.py
# Task s, remove the rancilio tool from the group head
# This is part l's insertion run backwards, the same way part j is part g run backwards. The empty gripper comes in
# the way part l left, down to the coupling of the locked tool, grabs it, twists it back round to the angle it went in
# at, drops it out of the group head and slides it out the front, the way part l brought it in. Every pose comes from
# part l's own functions with part l's values, so it meets the tool exactly where part l left it
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import run_visual
from _partL import compute_rancilio_pose, compute_task_l_pose, compute_task_l_twist_steps


def compute_task_s_poses(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg, handle_yaw_deg,
                         approach_offset_mm, group_head_tilt_deg, twist_deg, num_twist_steps):
    # Reuses part l's poses, so the tool is grabbed at exactly the position and angle part l left it at. Pass the
    # same values part l used. Returns the pose lined up below the group head, the pose at the entry angle, and the
    # poses through the twist, the last of which is the locked pose
    task_s_approach_pose = compute_task_l_pose(
        fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg, handle_yaw_deg, approach_offset_mm,
        group_head_tilt_deg)
    task_s_entry_pose = compute_task_l_pose(
        fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg, handle_yaw_deg,
        group_head_tilt_deg=group_head_tilt_deg)
    twist_steps = compute_task_l_twist_steps(
        task_s_entry_pose, fixture_fine_tune_mm, twist_deg, num_twist_steps, group_head_tilt_deg)

    return task_s_approach_pose, task_s_entry_pose, twist_steps


# Jogged joint waypoint that brings the empty gripper round towards the rancilio before lining up above the tool's
# coupling
approach_intermediate_pose_1 = [23.561060, -88.604205, 127.365805, -38.761601, 68.598596, 140.000000]


def run_task_s(UR5, tls, RDK):
    """ Does not return the robot home, ends holding the rancilio tool, lined up out in front of the rancilio, ready
    to swing over to the tool cleaner in task t """

    # ==================== FINE TUNING (edit these in the lab) ====================
    # The tool is grabbed where part l left it, so these MUST equal the values in run_task_l in _partLlefthand.py.
    # See there for what each one does.

    # ---------- Where the tool sits in the group head ----------
    fixture_fine_tune_mm = (0, 0, 0)
    group_head_tilt_deg = 1.7
    approach_angle_adjust_deg = (0, 0, 0)
    extra_tilt_deg = 0
    handle_yaw_deg = -45
    twist_deg = 45

    # ---------- The way part l went in and came out ----------
    approach_offset_mm = (0, 0, -20)
    num_twist_steps = 9
    front_of_group_head_mm = 150
    retreat_back_mm = 60
    retreat_up_mm = 100

    # The joint pose approach_intermediate_pose_1 is NOT affected by the settings above. If the arm comes at the pose
    # above the coupling the wrong way round, re-jog it.
    # =============================================================================

    # Computes the pose lined up below the group head, the pose at the entry angle, and the poses through the twist,
    # the last of which is the locked pose the tool was let go at
    task_s_approach_pose, task_s_entry_pose, twist_steps = compute_task_s_poses(
        fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg, handle_yaw_deg, approach_offset_mm,
        group_head_tilt_deg, twist_deg, num_twist_steps)
    locked_pose = twist_steps[-1]

    # The poses part l backed off the locked tool to, along the handle, then raised, the way in for this part
    mate_back_pose = locked_pose * transl(0, 0, -retreat_back_mm)
    mate_above_pose = transl(0, 0, retreat_up_mm) * mate_back_pose

    # The untwist, back through part l's twist steps in reverse, ending at the entry angle
    untwist_steps = list(reversed(twist_steps[:-1])) + [task_s_entry_pose]

    # The pose lined up in front of the rancilio, level with the approach pose, along the rancilio's +x, the same
    # place part l slid in from
    rancilio_pose = compute_rancilio_pose(fixture_fine_tune_mm)
    out_front = [rancilio_pose[i, 0] * front_of_group_head_mm for i in range(2)]
    front_of_group_head_pose = transl(out_front[0], out_front[1], 0) * task_s_approach_pose

    # Makes sure the poses below are read in the robot base frame with the master tool, as the assignment requires
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # Comes in the way part l left, above the tool's coupling, down, then straight in along the handle to mate with it
    UR5.MoveJ(approach_intermediate_pose_1, True)
    UR5.MoveJ(mate_above_pose, True)
    UR5.MoveL(mate_back_pose, True)
    UR5.MoveL(locked_pose, True)

    # Grabs the tool, hides the ghost in the group head, and shows it attached to the robot instead
    tls.student_tool_attach()
    run_visual(RDK, "Hide_Rancilio_Rancilio_Tool_Rotated")
    attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
    attached_tool.setVisible(True, False)

    # Twists the tool back round to the angle it went in at, to unlock it from the group head
    for step_pose in untwist_steps:
        UR5.MoveL(step_pose, True)

    # Drops the basket down out of the group head, then slides straight out the front the way part l brought it in
    UR5.MoveL(task_s_approach_pose, True)
    UR5.MoveL(front_of_group_head_pose, True)

    # Sends the robot home
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # Puts the rancilio tool in the group head and takes it off the tool stand, as it would be after task l.
    # Simulation only
    run_visual(RDK, "Show_Rancilio_Rancilio_Tool_Rotated")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    run_task_s(UR5, tls, RDK)
