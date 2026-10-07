# _partE.py
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import get_point, build_pose, rotate_about_tip, solve_frame_rotation, run_visual, enable_space_pause
import _partM as part_m

# locking torked out


def compute_task_e_poses(fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm):
    # Part b's calculation, see there for how it works
    origin_w = get_point(6)
    origin_l = get_point(27)
    ref_w = get_point(7)
    ref_l = get_point(28)

    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    mazzer_scale_pose = transl(*origin_w) * rotz(theta)

    mazzer_scale_pose = mazzer_scale_pose * transl(*fixture_fine_tune_mm)

    # The target is the lock lever (point 31), the tool's contact point is its tip (point 51)
    point_local = get_point(31)
    axis_mapping = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]

    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)

    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)

    # Lock pose, touching the lever
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_scale_pose * T_fixture_placement
    task_e_lock_pose = rotate_about_tip(T_world_target * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # Slide pose, the fixture frame shifted by slide_mm
    mazzer_scale_pose_slid = mazzer_scale_pose * transl(*slide_mm)
    T_world_target_slid = mazzer_scale_pose_slid * T_fixture_placement
    task_e_slide_pose = rotate_about_tip(T_world_target_slid * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # Press pose, shifted by press_mm on top of the slide
    mazzer_scale_pose_pressed = mazzer_scale_pose_slid * transl(*press_mm)
    T_world_target_pressed = mazzer_scale_pose_pressed * T_fixture_placement
    task_e_press_pose = rotate_about_tip(T_world_target_pressed * T_tool_offset.inv(), tool_point_local, angle_adjust)

    return task_e_lock_pose, task_e_slide_pose, task_e_press_pose


# Jogged joint waypoints for the lock approach and the locked config
unlock_approach_joints = [101.365767, -74.229626, 122.697049, -109.471706, -34.865600, -240.644986]
locked_retreat_pose = [102.667886, -69.211856, 117.847023, -111.306248, -34.250952, -240.644986]
staging_intermediate_pose = [118.506308, -97.117079, 116.960032, -109.759306, -89.525329, -206.644986]


def run_task_e(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================

    # Shifts all three poses together, along the scale's own axes
    fixture_fine_tune_mm = (-20, 0, 0)

    # Rotates the tool about its tip, the same as part b
    approach_angle_adjust_deg = (0, 60, -35)

    # How far the tip slides the lever, the opposite way to part b's unlock
    slide_mm = (35, 0, 0)

    # How far the tip then pushes past the slide pose to flip the lever back to locked
    press_mm = (0, 0, -11)

    # =============================================================================

    unlock_pose, slide_pose, press_pose = compute_task_e_poses(
        fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm)

    # The mazzer tool is already held, carried over from task d

    UR5.MoveJ(staging_intermediate_pose, True)

    # Moves to the lock lever, slides, then presses to lock the scale
    UR5.MoveJ(unlock_approach_joints, True)
    UR5.MoveL(unlock_pose, True)
    UR5.MoveL(slide_pose, True)
    UR5.MoveL(press_pose, True)
    run_visual(RDK, "Show_Mazzer_Scale_Lock")
    UR5.MoveL(locked_retreat_pose, True)

    UR5.MoveJ(staging_intermediate_pose, True)

    part_m.run_task_m(UR5, tls, RDK)

    # Returns the mazzer tool to the tool stand
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

    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # Shows the rancilio tool sitting on the mazzer scale and takes it off the tool stand, as task a leaves it, with
    # the scale unlocked, as task b leaves it. Simulation only
    run_visual(RDK, "Show_Mazzer_Scale_Rancilio_Tool")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)
    run_visual(RDK, "Show_Mazzer_Scale_Read")

    # Picks up the mazzer tool, in the full run it is already held, carried over from task d
    tls.mazzer_tool_attach_l_ati()

    run_task_e(UR5, tls, RDK)

    # Sends the robot home, in the full run task f carries straight on from the tool stand
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
