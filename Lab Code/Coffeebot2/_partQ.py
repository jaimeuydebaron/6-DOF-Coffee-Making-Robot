# _partQ.py
# Task q, use the mazzer tool to lock the rancilio scale
# This is part o run the other way, as part e is to part b. It comes in through part o's jogged poses, touches the
# same lever, then slides it along the scale's +x and presses down to flip it back to locked
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import compute_waypoints, run_visual, enable_space_pause
from _partO import (
    compute_task_o_poses, approach_intermediate_pose_3,
)


def compute_task_q_poses(fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm):
    # Part o's lever poses, only the slide and the press differ
    return compute_task_o_poses(fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm)


# Jogged joint waypoint that lines the tool up just before the straight move to this part's touch pose, which sits
# 30 mm along the scale's -x from part o's
o_approach_intermediate_pose_1 = [52.936614, -71.708468, 122.397036, -148.575562, -139.539322, -60.318949]


def run_task_q(UR5, tls, RDK):
    """ Does not return the robot home, starts holding the mazzer tool where part p ends, and ends with the mazzer
    tool back on the tool stand, at the left hand approach pose """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # Shifts all three poses together, along the scale's own axes
    fixture_fine_tune_mm = (-20, 0, 0)

    # Rotates the tool about its tip. Keep it close to part o's, so part o's jogged poses still line up
    approach_angle_adjust_deg = (0, -50, 130)

    # How far the tip slides the lever, the opposite way to part o's unlock
    slide_mm = (35, 0, 0)

    # How far the tip then pushes past the slide pose to flip the lever back to locked
    press_mm = (0, 0, -11)

    # How far the tip backs off along the tool's own z after the press
    backoff_mm = 20
    # =============================================================================

    # Computes the poses touching the lever, sliding it, and pressing it down
    touch_pose, slide_pose, press_pose = compute_task_q_poses(
        fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm)

    # The pose after pressing, backed straight off along the tool's own z
    waypoints = compute_waypoints(press_pose, offsets=[
        ("backoff", (0, 0, -backoff_mm), (0, 0, 0)),
    ])
    retreat_pose = waypoints["backoff"]

    # Reads the poses in the robot base frame with the master tool
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # The mazzer tool is already held, carried over from task p

    # Moves in through part o's jogged pose and this part's own, touches the lever, slides it, then presses it down to lock the scale
    UR5.MoveL(approach_intermediate_pose_3, True)
    UR5.MoveJ(o_approach_intermediate_pose_1, True)
    UR5.MoveL(touch_pose, True)
    UR5.MoveL(slide_pose, True)
    UR5.MoveL(press_pose, True)
    run_visual(RDK, "Show_Rancilio_Scale_Lock")

    # Backs straight off the lever and out through the jogged pose, then returns the mazzer tool, left handed
    UR5.MoveL(retreat_pose, True)
    UR5.MoveJ(approach_intermediate_pose_3, True)
    tls.mazzer_tool_detach_l_ati()


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

    # run_visual(RDK, "Reset_Simulation_L")

    # Shows what tasks l and n leave behind, the rancilio tool in the group head, off the tool stand, and the cup on
    # the scale pan. Simulation only
    run_visual(RDK, "Show_Rancilio_Rancilio_Tool_Rotated")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)
    run_visual(RDK, "Show_Rancilio_Scale_Cup")
    run_visual(RDK, "Show_Rancilio_Scale_Read")

    # Picks up the mazzer tool, in the full run it is already held, carried over from task p
    tls.mazzer_tool_attach_l_ati()

    run_task_q(UR5, tls, RDK)

    # Sends the robot home, in the full run task s carries straight on from the tool stand
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
