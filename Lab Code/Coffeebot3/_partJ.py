# _partJ.py
# Task j, open the wdt fixture, remove the rancilio tool and close the wdt fixture
# This is part g run backwards, it goes out the way g came in, and goes in the way g left, through the same
# jogged poses, so nothing new needs jogging
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import run_visual
from _partGH import (
    compute_task_g_pose,
    approach_intermediate_pose_1, wdt_approach_joints, wdt_pre_drop_joints,
    retreat_intermediate_pose_1, retreat_intermediate_pose_2,
)


def compute_task_j_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg=0):
    # Reuses part g's placement pose, so the tool is grabbed at exactly the position and angle part g left it at.
    # Pass the same values part g used.
    return compute_task_g_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg)


def run_task_j(UR5, tls, RDK):
    """ Does not return the robot home, __main.py chains straight into the next task """

    # ==================== FINE TUNING (edit these in the lab) ====================
    # This pose is where the robot grabs the rancilio tool back out of the wdt, so it must match where and how
    # part g left it. All three values MUST equal the ones in run_task_g in _partGH.py.

    # fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the robot goes to grab the tool, by shifting the assumed position of the wdt along the wdt's
    # own axes. The tool keeps its angle and just moves as a whole.
    #   x: shifts along the wdt's local x
    #   y: shifts along the wdt's local y
    #   z: shifts along the wdt's local z
    fixture_fine_tune_mm = (8, 0, 7)

    # approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z) at the grab pose.
    # It pivots about the tool's origin, NOT the ball bearing point, so changing it also moves where the tip lands.
    approach_angle_adjust_deg = (0, 0, 0)

    # extra_tilt_deg
    # Extra lean added on top of the 1.7 degree stem angle (point 63), about the tool's own y, pivoting about the
    # ball bearing point (point 54). The tool is grabbed at the same angle it was placed at.
    extra_tilt_deg = 0

    # The jogged joint poses are shared with part g and NOT affected by the settings above. wdt_pre_drop_joints is
    # the standoff the gripper moves in from before the final straight move to the grab pose, so if you move the
    # grab pose a lot, re-jog it so that last move stays clear of the wdt.
    # =============================================================================

    task_j_pose = compute_task_j_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg)

    # Reads the poses in the robot base frame with the master tool
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # Goes in the way part g left, out through the staging poses, to reach the wdt empty handed
    UR5.MoveJ(retreat_intermediate_pose_2, True)
    UR5.MoveJ(wdt_approach_joints, True)

    # Opens the wdt fixture before reaching in for the tool
    tls.wdt_open()

    UR5.MoveJ(retreat_intermediate_pose_1, True)
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

    RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

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
