# _partO.py
# Task o, use the mazzer tool to unlock the rancilio scale
# The rancilio scale is the same model as the mazzer scale, so this is part b on the other scale. The tool leans
# about its tip to come in under the handle of the rancilio tool locked in the group head
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import get_point, build_pose, rotate_about_tip, run_visual, enable_space_pause
from _partN import compute_rancilio_scale_pose



def compute_task_o_poses(fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm):
    rancilio_scale_pose = compute_rancilio_scale_pose(fixture_fine_tune_mm)

    # Part b's calculation on the rancilio scale's frame, the target is its lock lever (point 42)
    point_local = get_point(42)
    axis_mapping = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]

    # The tool's contact point is its tip (point 51)
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)

    # The angle adjust, applied about the tip below
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)

    # Touch pose, at the lever
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = rancilio_scale_pose * T_fixture_placement
    task_o_touch_pose = rotate_about_tip(T_world_target * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # Slide pose, the fixture frame shifted by slide_mm
    rancilio_scale_pose_slid = rancilio_scale_pose * transl(*slide_mm)
    T_world_target_slid = rancilio_scale_pose_slid * T_fixture_placement
    task_o_slide_pose = rotate_about_tip(T_world_target_slid * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # Press pose, shifted by press_mm on top of the slide
    rancilio_scale_pose_pressed = rancilio_scale_pose_slid * transl(*press_mm)
    T_world_target_pressed = rancilio_scale_pose_pressed * T_fixture_placement
    task_o_press_pose = rotate_about_tip(T_world_target_pressed * T_tool_offset.inv(), tool_point_local, angle_adjust)

    return task_o_touch_pose, task_o_slide_pose, task_o_press_pose


# Jogged joint waypoints that carry the mazzer tool in to the lever, left hand, clear of the rancilio tool's handle
# and the cup. The third is the furthest out, and the tool comes back out through it after the press
approach_intermediate_pose_1 = [52.227150, -69.439690, 116.216613, -144.100988, -134.526717, -60.386398]
approach_intermediate_pose_2 = [55.813531, -71.499284, 126.548408, -148.839072, -134.874286, -55.363720]
approach_intermediate_pose_3 = [49.032014, -109.144150, 153.150026, -154.043107, -132.893203, -64.360822]


def run_task_o(UR5, tls, RDK):
    """ Does not return the robot home, ends holding the mazzer tool in the left hand configuration, backed away
    from the lever at approach_intermediate_pose_3, ready for part p """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # Shifts all three poses together, along the scale's own axes
    fixture_fine_tune_mm = (10, 0, 0)

    # Rotates the tool about its tip, to lean in under the rancilio tool's handle. Part b's values mirrored for
    # this side, which flips the sign of the rotations about the tool's y and z
    approach_angle_adjust_deg = (0, -45, 130)

    # How far the tip slides the lever from the touch pose, the same as part b
    slide_mm = (-35, 0, 0)

    # How far the tip then pushes past the slide pose to flip the lever
    press_mm = (0, 0, -11)

    # How far the tip backs off along the tool's own z after the press
    backoff_mm = 80
    # =============================================================================

    # Computes the poses touching the lever, sliding it, and pressing it down
    touch_pose, slide_pose, press_pose = compute_task_o_poses(
        fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm)

    # The pose after pressing, backed straight off along the tool's own axis
    retreat_pose = press_pose * transl(0, 0, -backoff_mm)

    # Reads the poses in the robot base frame with the master tool
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

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
    UR5.MoveL(approach_intermediate_pose_3, True)


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

    run_task_o(UR5, tls, RDK)

    # Returns the mazzer tool and sends the robot home, in the full run task p carries on still holding it
    tls.mazzer_tool_detach_l_ati()
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
