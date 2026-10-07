# _partF.py
from robodk.robolink import *
from robodk.robomath import *
import tools
from _partA import tilt_tool_pose
from _common import run_visual, enable_space_pause


def compute_task_f_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg=0, hover_mm=0):
    # Part a's tilted placement pose, pass part a's values so the tool is grabbed where part a left it
    return tilt_tool_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm, extra_tilt_deg)


# Jogged waypoints for approaching the tool sitting on the mazzer scale, and
# for retreating clear of the mazzer body afterward
approach_intermediate_pose_1 = [118.506782, -100.274943, 101.666651, -91.307901, -89.525705, -206.644186]
approach_intermediate_pose_2 = [147.869587, -102.441040, 117.125449, -15.935730, 119.139464, -220.597282]
rancilio_align_intermediate_pose = [147.869438, -79.274625, 137.134086, -59.110450, 119.138938, -220.596797]

retreat_intermediate_pose_1 = [147.869438, -84.061570, 135.628719, -52.818138, 119.138938, -220.596797]


def run_task_f(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================
    # The grab pose must match where part a left the tool, so the first three MUST equal part a's values

    fixture_fine_tune_mm = (0, 0, -4)
    approach_angle_adjust_deg = (0, 0, 0)
    extra_tilt_deg = 0

    # How far the tool lifts straight up once grabbed, keeping its tilt, part a's drop run backwards
    align_hover_mm = 30

    # =============================================================================

    # The grab pose, and the pose hovering straight above it
    task_f_pose = compute_task_f_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg)
    task_f_hover_pose = compute_task_f_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg, align_hover_mm)

    # Makes sure the poses are read in the robot base frame with the master tool, a tool stand routine that was
    # stopped partway leaves the station in the tool stand's frame
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # Moves in to where the rancilio tool is sitting on the scale
    UR5.MoveJ(approach_intermediate_pose_1, True)
    UR5.MoveJ(approach_intermediate_pose_2, True)
    UR5.MoveL(rancilio_align_intermediate_pose, True)
    UR5.MoveL(task_f_pose, True)

    # # Grabs the tool, hides the ghost object, and shows it attached to the robot instead
    # attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
    # attached_tool.setVisible(True, False)
    # run_visual(RDK, "Hide_Mazzer_Scale_Rancilio_Tool")

    # tls.student_tool_attach()

    # # Lifts straight up to the hover pose, keeping the tilt, now holding the tool
    # UR5.MoveL(task_f_hover_pose, True)
    # UR5.MoveL(retreat_intermediate_pose_1, True)


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

    run_task_f(UR5, tls, RDK)

    # # Returns the rancilio tool and sends the robot home, in the full run task g carries on still holding it
    # tls.rancilio_tool_detach_l_ati()
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
