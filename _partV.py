# _partV.py
from robodk.robolink import *
from robodk.robomath import *
import tools
from _partS import run_task_s
from _partT import run_task_t, above_brush, BRISTLE_BRUSH
from _partU import run_task_u


def run_task_v(UR5, tls, RDK):
    # edit these in the lab

    # how far the tool is pulled back along the handle, towards the robot, before swinging over to the tool stand
    # without this the handle sweeps through the cup tool on the stand as the wrist turns
    pull_back_mm = 150

    # starts above the bristle brush where task u finishes, and pulls straight back along the handle (tool -z)
    UR5.MoveL(above_brush(BRISTLE_BRUSH) * transl(0, 0, -pull_back_mm), True)

    # already in the left hand configuration, so the tool stand's left hand detach routine
    # can swing over from here and return the rancilio tool
    tls.rancilio_tool_detach_l_ati()

    # sends the robot home, finishing the run
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)


if __name__ == "__main__":
    RDK = Robolink()

    # --- simulation settings (use when testing in robodk only) ---
    RDK.setRunMode(RUNMODE_SIMULATE)

    # --- real robot settings (uncomment when running on the physical robot) ---
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # connect to the robot before running: connect -> connect robot, then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)

    # sim only: puts the rancilio tool in the group head and takes it off the tool stand, as it would be after task l
    RDK.Item("Show_Rancilio_Rancilio_Tool_Rotated", ITEM_TYPE_PROGRAM).RunCode()
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    # tasks t, u and v carry on from where task s finishes, so all four run back to back
    run_task_s(UR5, tls, RDK)
    run_task_t(UR5, tls, RDK)
    run_task_u(UR5, tls, RDK)
    run_task_v(UR5, tls, RDK)
