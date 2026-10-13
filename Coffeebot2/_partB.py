# _partB.py
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import get_point, build_pose, rotate_about_tip, solve_frame_rotation, run_visual, enable_space_pause

# The lock lever calculation, also used by parts e, o and q. It follows part a's steps with two differences
#   The angle adjust goes through rotate_about_tip, so the tool pivots about its tip (point 51). The tip stays on
#   the lever while the tool leans clear of whatever is hanging over the scale
#   Three poses are built by shifting the fixture frame along the scale's own axes: lock (touching the lever),
#   slide (shifted by slide_mm), and press (shifted by press_mm on top of the slide)

def compute_task_b_poses(fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm):
    # The mazzer scale's frame, see part a
    origin_w = get_point(6)
    origin_l = get_point(27)
    ref_w = get_point(7)
    ref_l = get_point(28)

    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    mazzer_scale_pose = transl(*origin_w) * rotz(theta)

    # Fine tune
    mazzer_scale_pose = mazzer_scale_pose * transl(*fixture_fine_tune_mm)

    # The target is the lock lever (point 31), the tool's contact point is its tip (point 51)
    point_local = get_point(31)
    axis_mapping = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]

    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)

    # The angle adjust, applied about the tip below
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)

    # Lock pose, touching the lever
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_scale_pose * T_fixture_placement
    task_b_lock_pose = rotate_about_tip(T_world_target * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # Slide pose, the fixture frame shifted by slide_mm
    mazzer_scale_pose_slid = mazzer_scale_pose * transl(*slide_mm)
    T_world_target_slid = mazzer_scale_pose_slid * T_fixture_placement
    task_b_slide_pose = rotate_about_tip(T_world_target_slid * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # Press pose, shifted by press_mm on top of the slide
    mazzer_scale_pose_pressed = mazzer_scale_pose_slid * transl(*press_mm)
    T_world_target_pressed = mazzer_scale_pose_pressed * T_fixture_placement
    task_b_press_pose = rotate_about_tip(T_world_target_pressed * T_tool_offset.inv(), tool_point_local, angle_adjust)

    return task_b_lock_pose, task_b_slide_pose, task_b_press_pose


# Jogged joint waypoints for the unlock approach and the unlocked config,
# tuned so the arm avoids the rancilio tool already sitting on the scale
staging_intermediate_pose = [118.506308, -97.117079, 116.960032, -109.759306, -89.525329, -206.644986]
locked_approach_pose = [102.490000, -70.890000, 118.190000, -109.750000, -34.330000, -256.64]
unlocked_retreat_pose = [101.163578, -73.565929, 123.890487, -111.075789, -34.964000, -244.38]


def run_task_b(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================

    # Shifts all three poses together, along the scale's own axes
    fixture_fine_tune_mm = (10, 0, 0)

    # Rotates the tool about its tip, here to lean it clear of the rancilio tool sitting on the scale
    approach_angle_adjust_deg = (0, 60, -35)

    # How far the tip slides the lever from the lock pose, along the scale's own axes
    slide_mm = (-35, 0, 0)

    # How far the tip then pushes past the slide pose to flip the lever, bigger if it doesn't flip
    press_mm = (0, 0, -11)

    # =============================================================================

    lock_pose, slide_pose, press_pose = compute_task_b_poses(
        fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm)

    # Picks up the mazzer tool from the tool stand
    tls.mazzer_tool_attach_l_ati()

    UR5.MoveJ(staging_intermediate_pose, True)

    # Moves to the lock lever, slides, then presses to unlock the scale
    UR5.MoveJ(locked_approach_pose, True)
    UR5.MoveL(lock_pose, True)
    UR5.MoveL(slide_pose, True)
    UR5.MoveL(press_pose, True)
    run_visual(RDK, "Show_Mazzer_Scale_Read")
    UR5.MoveL(unlocked_retreat_pose, True)
    UR5.MoveJ(staging_intermediate_pose, True)

    # Ends still holding the mazzer tool, for task c


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

    # Shows the rancilio tool sitting on the mazzer scale and takes it off the tool stand, as task a leaves it.
    # Simulation only
    run_visual(RDK, "Show_Mazzer_Scale_Rancilio_Tool")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    run_task_b(UR5, tls, RDK)

    # Returns the mazzer tool and sends the robot home, in the full run task c carries on still holding it
    tls.mazzer_tool_detach_l_ati()
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
