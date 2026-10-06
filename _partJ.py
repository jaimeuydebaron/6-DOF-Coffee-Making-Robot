# _partJ.py
# Task j, open the wdt fixture, remove the rancilio tool and close the wdt fixture
# This is part g run backwards, it goes out the way g came in, and goes in the way g left, through the same
# jogged poses, so nothing new needs jogging
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import run_visual, enable_space_pause
from _partGH import (
    compute_task_g_pose,
    wdt_approach_joints, wdt_pre_drop_joints, retreat_intermediate_pose_2,
)


def compute_task_j_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg=0):
    # Part g's placement pose, pass part g's values so the tool is grabbed where part g left it
    return compute_task_g_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg)

wdt_apprpoach_intermediate_pose_1 = [136.540486, -80.982847, 143.019227, -67.127780, 46.654059, -215.500306]

def run_task_j(UR5, tls, RDK):
    """ Does not return the robot home, __main.py chains straight into the next task """

    # ==================== FINE TUNING (edit these in the lab) ====================
    # The grab pose must match where part g left the tool, so all three MUST equal the values in run_task_g

    fixture_fine_tune_mm = (8, 0, 7)
    approach_angle_adjust_deg = (0, 0, 0)
    extra_tilt_deg = 0

    # =============================================================================

    task_j_pose = compute_task_j_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg)

    # Goes in the way part g left, out through the staging poses, to reach the wdt empty handed
    UR5.MoveJ(retreat_intermediate_pose_2, True)

    # Opens the wdt fixture before reaching in for the tool
    tls.wdt_open()

    UR5.MoveJ(wdt_apprpoach_intermediate_pose_1, True)
    UR5.MoveL(task_j_pose, True)

    # Grabs the tool, hides the ghost object in the wdt, and shows it attached to the robot instead
    attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
    attached_tool.setVisible(True, False)
    run_visual(RDK, "Hide_WDT_Rancilio_Tool")

    tls.student_tool_attach()

    # Takes the tool out the way part g brought it in, then closes the wdt fixture with the tool clear of it
    UR5.MoveL(wdt_pre_drop_joints, True)
    UR5.MoveJ(wdt_approach_joints, True)

    tls.wdt_shut()


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

    # Shows the rancilio tool sitting in the wdt and takes it off the tool stand, as tasks g and h leave it.
    # Simulation only
    run_visual(RDK, "Show_WDT_Rancilio_Tool")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    # Starts at home, where tasks g and h finish
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)

    run_task_j(UR5, tls, RDK)

    # Returns the rancilio tool and sends the robot home, in the full run task k carries on still holding it
    tls.rancilio_tool_detach_l_ati()
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
