# _partO.py
# Task o, use the mazzer tool to unlock the rancilio scale
# The rancilio scale is the same model as the mazzer scale, its lock lever points (42, 43) match points 30 and 31
# exactly, so this unlocks it the same way part b does, the tool tip touches the lever, slides it along the scale's
# -x, then presses down to flip it. The difference is the rancilio tool, locked in the group head by part l, whose
# handle hangs out over the scale, so the tool is turned about its tip to come in under it, the same way part b
# does for the rancilio tool sitting on the mazzer scale. Like part b, it is all in the left hand configuration
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, rotate_about_tip, compute_waypoints, run_visual
from _partN import compute_rancilio_scale_pose


# Arm configuration hints for joint moves solved near the rancilio scale. Neither is used by this part, which comes
# in through jogged poses, but parts p and q import them. They only pick which of the arm's solutions is used, they
# do not change where the tool goes
# Left hand with the elbow and the wrist up, the same way round as tasks l, m and n
LEFT_LEVER_CONFIG_HINT = [11, -100, 144, -44, 101, 140]
# Right hand with the elbow up
LEVER_CONFIG_HINT = [-120, -100, -110, -150, 90, -220]


def compute_task_o_poses(fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm):
    rancilio_scale_pose = compute_rancilio_scale_pose(fixture_fine_tune_mm)

    # Sets the placement point as the left lock lever (point 42, the same spot as part b's point 30), with the tool
    # orientation found by inspection, the same as part b, the tool pointing straight down at the lever
    point_local = get_point(42)
    axis_mapping = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]

    # Sets the mazzer tool's contact point, the cushioning cylinder tip (point 51)
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)

    # Builds the angle correction, rotated about the tool tip so its position doesn't move
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)

    # Builds the touch pose at the lock lever
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = rancilio_scale_pose * T_fixture_placement
    task_o_touch_pose = rotate_about_tip(T_world_target * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # Builds the slide pose by offsetting the fixture pose along its own local frame
    rancilio_scale_pose_slid = rancilio_scale_pose * transl(*slide_mm)
    T_world_target_slid = rancilio_scale_pose_slid * T_fixture_placement
    task_o_slide_pose = rotate_about_tip(T_world_target_slid * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # Builds the press pose by offsetting further along the fixture's own frame
    rancilio_scale_pose_pressed = rancilio_scale_pose_slid * transl(*press_mm)
    T_world_target_pressed = rancilio_scale_pose_pressed * T_fixture_placement
    task_o_press_pose = rotate_about_tip(T_world_target_pressed * T_tool_offset.inv(), tool_point_local, angle_adjust)

    return task_o_touch_pose, task_o_slide_pose, task_o_press_pose


# Jogged joint waypoints that carry the mazzer tool in to the lever, left hand, clear of the rancilio tool's handle
# and the cup. The third is the furthest out, and the tool comes back out through it after the press
approach_intermediate_pose_1 = [52.227150, -69.439690, 116.216613, -144.100988, -134.526717, -60.386398]
approach_intermediate_pose_2 = [55.813531, -71.499284, 126.548408, -148.839072, -134.874286, -55.363720]
approach_intermediate_pose_3 = [55.810000, -101.500000, 146.550000, -148.840000, -134.870000, -55.360000]


def run_task_o(UR5, tls, RDK):
    """ Does not return the robot home, ends holding the mazzer tool in the left hand configuration, backed away
    from the lever at approach_intermediate_pose_3, ready for part p """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # fixture_fine_tune_mm = (x, y, z) in mm
    # Moves ALL THREE poses (touch, slide, press) together, by shifting the assumed position of the rancilio scale
    # along the scale's own axes. Use it when the tool misses the lever from the very first contact.
    #   x: shifts along the scale's local x
    #   y: shifts along the scale's local y
    #   z: shifts along the scale's local z
    fixture_fine_tune_mm = (10, 0, 0)

    # approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z) for the touch, slide and press poses.
    # It pivots about the tool tip (point 51), so the tip stays on the lever and only the tool's facing direction
    # changes. Use it to change how the tool comes at the lever, e.g. to clear the rancilio tool whose handle hangs
    # over the scale. Started from part b's values mirrored for the left lever, the tool's x axis is the scale's y
    # axis, so mirroring the approach to the other side flips the sign of the rotations about the tool's y and z.
    # Part q MUST use the same value, so it comes at the lever the same way.
    approach_angle_adjust_deg = (0, -45, 130)

    # slide_mm = (x, y, z) in mm
    # How far the tool slides from the touch pose along the scale's own axes, before pressing the lever.
    # Changes only the slide pose (and the press pose, which is built on top of it).
    # Default -35 in x slides 35 mm along the scale's -x, the same as part b.
    slide_mm = (-35, 0, 0)

    # press_mm = (x, y, z) in mm
    # How far the tool then pushes past the slide pose, along the scale's own axes, to flip the lever.
    # Changes only the press pose. It adds on top of slide_mm. Default -10 in z pushes 10 mm along the scale's -z,
    # make it bigger if the lever doesn't flip, smaller if the tool pushes too hard.
    press_mm = (0, 0, -10)

    # backoff_mm
    # How far back along the tool's own z the tip backs off to after pressing the lever, before heading out through
    # the jogged poses. Bigger = more clearance from the rancilio tool's handle, smaller = a shorter move.
    backoff_mm = 80

    # The joint poses above (approach_intermediate_pose_1/2/3) are jogged joint angles, not affected by the settings
    # here. approach_intermediate_pose_1 is where the straight move to the lever starts from, so if you move the
    # fixture or change the angle a lot, re-jog it so that move stays clear of the rancilio tool and the cup.
    # =============================================================================

    # Computes the poses touching the lever, sliding it, and pressing it down
    touch_pose, slide_pose, press_pose = compute_task_o_poses(
        fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm)

    # The pose after pressing, backed straight off along the tool's own axis
    retreat_pose = press_pose * transl(0, 0, -backoff_mm)

    # Makes sure the poses below are read in the robot base frame with the master tool, as the assignment requires
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # Optional, builds extra waypoints from the touch pose instead of using the jogged poses
    # Each offset is (name, (x, y, z) in mm, (rx, ry, rz) in degrees), applied in the tool's OWN frame,
    # translation first then rotation. Each waypoint comes from touch_pose on its own, they do not chain.
    # For example, this backs the tool off 30 mm along its own z and 80 mm along its own -x:
    waypoints = compute_waypoints(touch_pose, offsets=[
        ("backoff", (-80, 0, -30), (0, 0, 0)),
    ])
    # UR5.MoveL(waypoints["backoff"], True)
    # UR5.MoveL(touch_pose, True)

    # Picks up the mazzer tool from the tool stand, left handed
    tls.mazzer_tool_attach_l_ati()

    # Moves in through the jogged poses, touches the lever, slides it, then presses it down to unlock the scale
    UR5.MoveJ(approach_intermediate_pose_3, True)
    UR5.MoveJ(approach_intermediate_pose_2, True)
    UR5.MoveJ(approach_intermediate_pose_1, True)
    UR5.MoveL(touch_pose, True)
    UR5.MoveL(slide_pose, True)
    UR5.MoveL(press_pose, True)
    run_visual(RDK, "Show_Rancilio_Scale_Read")

    # Backs straight off the lever and out through the jogged pose, still holding the mazzer tool for part p
    UR5.MoveL(retreat_pose, True)
    UR5.MoveJ(approach_intermediate_pose_3, True)

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

    # Resets the simulation, then shows what parts l and n leave behind, the rancilio tool in the group head and
    # the cup on the scale pan. Simulation only, comment these out on the real robot
    # run_visual(RDK, "Reset_Simulation_L")
    run_visual(RDK, "Show_Rancilio_Rancilio_Tool_Rotated")
    run_visual(RDK, "Show_Rancilio_Scale_Cup")

    run_task_o(UR5, tls, RDK)
