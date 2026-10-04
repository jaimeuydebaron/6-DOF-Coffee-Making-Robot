# _partE.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, rotate_about_tip, solve_frame_rotation, compute_waypoints

# locking torked out


def compute_task_e_poses(fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm):
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

    # Sets the placement point as the left lock lever, with the tool orientation found by inspection
    point_local = get_point(31)
    axis_mapping = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]

    # Sets the mazzer tool's contact point
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)

    # Builds the angle correction, rotated about the tool tip so its position doesn't move
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)

    # Builds the approach pose at the lock lever
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_scale_pose * T_fixture_placement
    task_e_lock_pose = rotate_about_tip(T_world_target * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # Builds the slide pose by offsetting the fixture pose along its own local frame
    mazzer_scale_pose_slid = mazzer_scale_pose * transl(*slide_mm)
    T_world_target_slid = mazzer_scale_pose_slid * T_fixture_placement
    task_e_slide_pose = rotate_about_tip(T_world_target_slid * T_tool_offset.inv(), tool_point_local, angle_adjust)

    # Builds the press pose by offsetting further along the fixture's own frame
    mazzer_scale_pose_pressed = mazzer_scale_pose_slid * transl(*press_mm)
    T_world_target_pressed = mazzer_scale_pose_pressed * T_fixture_placement
    task_e_press_pose = rotate_about_tip(T_world_target_pressed * T_tool_offset.inv(), tool_point_local, angle_adjust)

    return task_e_lock_pose, task_e_slide_pose, task_e_press_pose


# Jogged joint waypoints for the lock approach and the locked config
unlock_approach_joints = [101.365767, -74.229626, 122.697049, -109.471706, -34.865600, 15.962565]
locked_retreat_pose = [102.667886, -69.211856, 117.847023, -111.306248, -34.250952, 17.986387]
staging_intermediate_pose = [118.506308, -97.117079, 116.960032, -109.759306, -89.525329, -206.644986]


def run_task_e(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================

    # fixture_fine_tune_mm = (x, y, z) in mm
    # Moves ALL THREE poses (lock, slide, press) together, by shifting the assumed position of the mazzer scale
    # along the scale's own axes. Use it when the tool misses the lever from the very first contact.
    #   x: shifts along the scale's local x
    #   y: shifts along the scale's local y
    #   z: shifts along the scale's local z
    fixture_fine_tune_mm = (-20, 0, 0)

    # approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z) for the lock, slide and press poses.
    # It pivots about the tool tip (point 51), so the tip stays on the lever and only the tool's facing direction
    # changes. Use it to change how the tool comes at the lever, e.g. to clear the rancilio tool sitting on the scale.
    approach_angle_adjust_deg = (0, 60, -130)

    # slide_mm = (x, y, z) in mm
    # How far the tool slides from the lock pose along the scale's own axes, before pressing the lever.
    # Changes only the slide pose (and the press pose, which is built on top of it).
    # Default 15 in x slides 15 mm along the scale's +x, the opposite direction to part B's unlock slide.
    slide_mm = (35, 0, 0)

    # press_mm = (x, y, z) in mm
    # How far the tool then pushes past the slide pose, along the scale's own axes, to flip the lever back to locked.
    # Changes only the press pose. It adds on top of slide_mm. Default -15 in z pushes 15 mm along the scale's -z,
    # make it bigger if the lever doesn't flip, smaller if the tool pushes too hard.
    press_mm = (0, 0, -10)

    # The joint poses below (staging_intermediate_pose, lock_approach_joints, lock_locked_joints) are jogged joint
    # angles, not affected by the settings above. lock_locked_joints is where the tool retreats to after the press,
    # so if you change press_mm by a lot, re-jog it. Re-jog the others too if the arm gets close to the rancilio
    # tool after moving the fixture.
    # =============================================================================

    unlock_pose, slide_pose, press_pose = compute_task_e_poses(
        fixture_fine_tune_mm, approach_angle_adjust_deg, slide_mm, press_mm)

    # Optional, builds extra waypoints from the final tilted pose instead of using align_hover_mm
    # Each offset is (name, (x, y, z) in mm, (rx, ry, rz) in degrees), applied in the tool's OWN frame,
    # translation first then rotation. Each waypoint comes from task_a_pose on its own, they do not chain.
    # For example, this backs the tool off 40 mm along its own z, keeping the tilt:
    waypoints = compute_waypoints(press_pose, offsets=[
        ("backoff", (0, 0, -30), (0, 0, 0)),
    ])
    # UR5.MoveL(waypoints["backoff"], True)
    # UR5.MoveL(task_a_pose, True)

    # Starts the robot at home
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
    
    # Picks up the mazzer tool from the tool stand
    # tls.mazzer_tool_attach_l_ati()

    UR5.MoveJ(staging_intermediate_pose, True)

    # Moves to the lock lever, slides, then presses to lock the scale
    UR5.MoveJ(unlock_approach_joints, True)
    UR5.MoveL(unlock_pose, True)
    UR5.MoveL(slide_pose, True)
    UR5.MoveL(press_pose, True)
    UR5.MoveL(locked_retreat_pose, True)

    UR5.MoveJ(staging_intermediate_pose, True)

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
    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    run_task_e(UR5, tls, RDK)


