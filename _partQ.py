# _partQ.py
# Task q, use the mazzer tool to lock the rancilio scale
# This is part o run the other way, the same way part e is part b run the other way. It comes in through part o's
# jogged poses in the left hand configuration, touches the same lever at the same angle, then slides it along the
# scale's +x and presses down to flip it back to locked
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import compute_waypoints, run_visual
from _partO import (
    compute_task_o_poses,
    approach_intermediate_pose_1, approach_intermediate_pose_2, approach_intermediate_pose_3,
)


def compute_task_q_poses(fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm):
    # Reuses part o's lever poses, so the tool comes at the lever the same way part o does. Only the slide and the
    # press are different, they push the lever back the other way
    return compute_task_o_poses(fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm)


# Jogged joint waypoint that lines the tool up just before the straight move to this part's touch pose, which sits
# 30 mm along the scale's -x from part o's
o_approach_intermediate_pose_1 = [52.936614, -71.708468, 122.397036, -148.575562, -139.539322, -60.318949]


def run_task_q(UR5, tls, RDK):
    """ Does not return the robot home, starts holding the mazzer tool where part p ends, and ends with the mazzer
    tool back on the tool stand, at the left hand approach pose """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # fixture_fine_tune_mm = (x, y, z) in mm
    # Moves ALL THREE poses (touch, slide, press) together, by shifting the assumed position of the rancilio scale
    # along the scale's own axes. Use it when the tool misses the lever from the very first contact.
    # Starts from part e's value, which locks the mazzer scale's matching lever.
    #   x: shifts along the scale's local x
    #   y: shifts along the scale's local y
    #   z: shifts along the scale's local z
    fixture_fine_tune_mm = (-20, 0, 0)

    # approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z) for the touch, slide and press poses.
    # It pivots about the tool tip (point 51), so the tip stays on the lever and only the tool's facing direction
    # changes. MUST equal part o's approach_angle_adjust_deg, so the tool leans in under the rancilio tool's handle
    # the same way part o does and part o's jogged poses still line up with it.
    approach_angle_adjust_deg = (0, -50, 130)

    # slide_mm = (x, y, z) in mm
    # How far the tool slides from the touch pose along the scale's own axes, before pressing the lever.
    # Default 35 in x slides 35 mm along the scale's +x, the opposite direction to part o's unlock slide, the same
    # as part e.
    slide_mm = (35, 0, 0)

    # press_mm = (x, y, z) in mm
    # How far the tool then pushes past the slide pose, along the scale's own axes, to flip the lever back to locked.
    # It adds on top of slide_mm. Default -11 in z pushes 11 mm along the scale's -z, the same as part e.
    press_mm = (0, 0, -11)

    # backoff_mm
    # How far back along the tool's own z the tip backs off to after pressing the lever, before heading out through
    # the jogged poses. Bigger = more clearance from the rancilio tool's handle, smaller = a shorter move.
    backoff_mm = 20

    # The joint poses (approach_intermediate_pose_1/2/3) are part o's jogged joint angles, imported from _partO.py
    # and NOT affected by the settings here. approach_intermediate_pose_1 is where the straight move to the lever
    # starts from. It was jogged for part o's touch pose, which sits 30 mm further along the scale's +x than this
    # one, so if the move in clips the rancilio tool or the cup, re-jog it in _partO.py.
    # =============================================================================

    # Computes the poses touching the lever, sliding it, and pressing it down
    touch_pose, slide_pose, press_pose = compute_task_q_poses(
        fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm)

    # The pose after pressing, backed straight off along the tool's own z
    waypoints = compute_waypoints(press_pose, offsets=[
        ("backoff", (0, 0, -backoff_mm), (0, 0, 0)),
    ])
    retreat_pose = waypoints["backoff"]

    # Makes sure the poses below are read in the robot base frame with the master tool, as the assignment requires
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # The mazzer tool is already held, carried over from task p, uncomment this only to test part q on its own
    # tls.mazzer_tool_attach_l_ati()

    # Moves in through part o's jogged pose and this part's own, touches the lever, slides it, then presses it down to lock the scale
    UR5.MoveJ(approach_intermediate_pose_3, True)
    UR5.MoveJ(o_approach_intermediate_pose_1, True)
    UR5.MoveL(touch_pose, True)
    UR5.MoveL(slide_pose, True)
    UR5.MoveL(press_pose, True)
    run_visual(RDK, "Show_Rancilio_Scale_Lock")

    # Backs straight off the lever and out through the jogged pose, then returns the mazzer tool, left handed
    UR5.MoveL(retreat_pose, True)
    UR5.MoveJ(approach_intermediate_pose_3, True)
    tls.mazzer_tool_detach_l_ati()

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

    # Resets the simulation, then shows what parts l and n leave behind, the rancilio tool in the group head and
    # the cup on the scale pan. Simulation only, comment these out on the real robot
    # run_visual(RDK, "Reset_Simulation_L")
    run_visual(RDK, "Show_Rancilio_Rancilio_Tool_Rotated")
    run_visual(RDK, "Show_Rancilio_Scale_Cup")

    run_task_q(UR5, tls, RDK)
