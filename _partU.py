# _partU.py
from robodk.robolink import *
from robodk.robomath import *
import tools
from _partS import run_task_s
from _partT import run_task_t, above_brush, clean_on_brush, BRISTLE_BRUSH


def run_task_u(UR5, tls, RDK):
    # starts above the silicone brush where task t finishes, and slides straight across
    # at the travel height to line the basket up above the bristle brush
    UR5.MoveL(above_brush(BRISTLE_BRUSH), True)

    # lowers onto the bristle brush, presses to actuate the cleaner for 5 s, then lifts clear
    clean_on_brush(UR5, BRISTLE_BRUSH)

    # sends the robot home holding the tool, ready to return it to the tool stand in task v
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)


if __name__ == "__main__":
    RDK = Robolink()

    # --- simulation settings (use when testing in robodk only) ---
    RDK.setRunMode(RUNMODE_SIMULATE)

    # --- real robot settings (uncomment when running on the physical robot) ---
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # connect to the robot before running: connect -> connect robot, then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # sim only: puts the rancilio tool in the group head and takes it off the tool stand, as it would be after task l
    RDK.Item("Show_Rancilio_Rancilio_Tool_Rotated", ITEM_TYPE_PROGRAM).RunCode()
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    # tasks t and u carry on from where task s finishes, so all three run back to back
    run_task_s(UR5, tls, RDK)
    run_task_t(UR5, tls, RDK)
    run_task_u(UR5, tls, RDK)
