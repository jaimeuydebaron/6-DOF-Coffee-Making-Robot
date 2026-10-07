# _partC.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, compute_mazzer_pose, run_visual, enable_space_pause


def compute_task_c_on_pose(fixture_fine_tune_mm, approach_angle_adjust_deg):
    # Same steps as part a, on the mazzer's frame
    mazzer_pose = compute_mazzer_pose()
    mazzer_pose = mazzer_pose * transl(*fixture_fine_tune_mm)

    # The target is the on button (point 26), the tool's contact point is its tip (point 51)
    point_local = get_point(26)
    axis_mapping = [[0, 1, 0], [0, 0, -1], [-1, 0, 0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    task_c_on_pose = T_world_target * T_tool_offset.inv()

    # Angle adjust about the tcp
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_c_on_pose = task_c_on_pose * angle_adjust

    return task_c_on_pose


def compute_task_c_off_pose(fixture_fine_tune_mm, approach_angle_adjust_deg):
    mazzer_pose = compute_mazzer_pose()
    mazzer_pose = mazzer_pose * transl(*fixture_fine_tune_mm)

    # Same as the on pose, at the off button (point 25)
    point_local = get_point(25)
    axis_mapping = [[0, 1, 0], [0, 0, -1], [-1, 0, 0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    task_c_off_pose = T_world_target * T_tool_offset.inv()

    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_c_off_pose = task_c_off_pose * angle_adjust

    return task_c_off_pose


# Jogged pose that clears the mazzer body on the way in from home
staging_intermediate_pose = [118.506308, -97.117079, 116.960032, -109.759306, -89.525329, -206.644986]
approach_intermediate_pose_1 = [98.510000, -97.120000, 116.960000, -109.760000, -89.530000, -206.640000]

# Joint waypoint that ensures the on button is approached at the right angle
on_button_approach_pose = [99.408373, -61.514148, 105.709453, -40.390944, 22.650255, -206.640000]

# Joint waypoint that ensures the off button is approached at the right angle
off_button_approach_pose = [99.693733, -59.564093, 102.529120, -39.203535, 22.933805, 105.284723]


def run_task_c(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================
    # The on and off buttons are tuned separately

    # Shifts where the tip presses the on button, along the mazzer's own axes
    on_fixture_fine_tune_mm = (-32, -20, 52)

    # Rotates the tool about the tcp, so the tip moves too, re-tune the fine tune after changing it
    on_approach_angle_adjust_deg = (-40, 15, 0)

    # The same two settings for the off button
    off_fixture_fine_tune_mm = (-30, -18, 48)
    off_approach_angle_adjust_deg = (-40, 15, 0)

    # =============================================================================

    # Computes the final poses for pressing the on and off buttons
    on_pose = compute_task_c_on_pose(on_fixture_fine_tune_mm, on_approach_angle_adjust_deg)
    off_pose = compute_task_c_off_pose(off_fixture_fine_tune_mm, off_approach_angle_adjust_deg)

    # The mazzer tool is already held, carried over from task b

    # Moves in, through a jogged joint waypoint, to press the on button
    UR5.MoveJ(staging_intermediate_pose, True)
    UR5.MoveJ(approach_intermediate_pose_1, True)
    UR5.MoveJ(on_button_approach_pose, True)
    UR5.MoveL(on_pose, True)
    UR5.MoveJ(on_button_approach_pose, True)

    # sleep(3)  # Grind time, currently a placeholder for testing rather than the real brew length
    sleep(15) # Actual grind time

    # Moves to press the off button, then through a jogged joint waypoint to retreat safely
    UR5.MoveL(off_button_approach_pose, True)
    UR5.MoveL(off_pose, True)
    UR5.MoveL(off_button_approach_pose, True)

    # Retreats, still holding the mazzer tool for task d
    UR5.MoveJ(approach_intermediate_pose_1, True)
    UR5.MoveJ(staging_intermediate_pose, True)


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

    # Picks up the mazzer tool, in the full run it is already held, carried over from task b
    tls.mazzer_tool_attach_l_ati()

    run_task_c(UR5, tls, RDK)

    # Returns the mazzer tool and sends the robot home, in the full run task d carries on still holding it
    tls.mazzer_tool_detach_l_ati()
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
