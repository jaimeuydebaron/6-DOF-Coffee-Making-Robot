# _partV.py
# Task v, return the rancilio tool to the tool stand
# Pulls back along the handle from above the bristle brush, then the tool stand's left hand detach routine takes over
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import run_visual
from _partU import compute_task_u_pose


def run_task_v(UR5, tls, RDK):
    """ Does not return the robot home, starts above the bristle brush where task u finishes, and ends empty handed
    with the rancilio tool back on the tool stand, at the left hand approach pose """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # These three MUST equal the values in run_task_u in _partU.py, so the pull back starts from where task u
    # finished, above the bristle brush. See run_task_t in _partT.py for what each one does.
    fixture_fine_tune_mm = (0, 0, 0)
    approach_angle_adjust_deg = (0, 0, 0)
    approach_height_mm = 100

    # pull_back_mm
    # How far the tool is pulled back along the handle, towards the robot, before swinging over to the tool stand.
    # Without this the handle sweeps through the cup tool on the stand as the wrist turns.
    pull_back_mm = 150
    # =============================================================================

    # Computes the pose above the bristle brush, pulled straight back along the handle (the tool's -z)
    hover_pose = compute_task_u_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, approach_height_mm)
    pulled_back_pose = hover_pose * transl(0, 0, -pull_back_mm)

    # Pulls straight back along the handle, clear of the cup tool on the stand
    UR5.MoveL(pulled_back_pose, True)

    # Already in the left hand configuration, so the tool stand's left hand detach routine can swing over from here
    # and return the rancilio tool
    tls.rancilio_tool_detach_l_ati()

    # Sends the robot home, the end of the whole run
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)


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

    # Puts the rancilio tool in the group head and takes it off the tool stand, as it would be after task l.
    # Simulation only
    run_visual(RDK, "Show_Rancilio_Rancilio_Tool_Rotated")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    run_task_v(UR5, tls, RDK)
