# _partQ.py
# Task q, use the mazzer tool to lock the rancilio scale
# The same lever and the same tool lean as part o, which unlocks it, but the lever is slid the other way along the
# scale's +x and pressed down, the same way part e locks the mazzer scale. The tool leans in under the rancilio
# tool's handle, which hangs out over the scale from the group head
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import solve_joints_near
from _partN import run_visual
from _partO import compute_task_o_pose, LEVER_CONFIG_HINT


def run_task_q(UR5, tls, RDK):
    """ Starts holding the mazzer tool backed off the switch where part p ends, and ends with the mazzer tool
    back on the tool stand, at the right hand approach pose, ready for part r """

    # ==================== FINE TUNING (edit these in the lab) ====================
    fixture_fine_tune_mm = (-20, 0, 0)  # moves every pose, along the scale's axes, the same start as part e
    tilt_deg = 60                       # how far the tool leans over from straight down, the same as part o
    lean_toward_deg = 180               # which way it leans, in the scale's frame, 180 is back towards the robot
    spin_deg = 0                        # spins the tool about its own axis, only moves the dose bar out of the way
    slide_mm = (35, 0, 0)               # how far the tip slides the lever, along the scale's +x, the same as part e
    press_mm = (0, 0, -11)              # then how far it presses down past the slide, the same as part e
    backoff_mm = 80                     # how far back along the tool the tip lines up before, and backs off to after
    # =============================================================================

    def lever(offset_mm):
        return compute_task_o_pose(fixture_fine_tune_mm, tilt_deg, lean_toward_deg, spin_deg, offset_mm)

    touch_pose = lever((0, 0, 0))
    slide_pose = lever(slide_mm)
    press_pose = lever(tuple(s + p for s, p in zip(slide_mm, press_mm)))

    # The poses lined up before touching and after pressing, backed straight off along the tool's own axis
    approach_pose = touch_pose * transl(0, 0, -backoff_mm)
    retreat_pose = press_pose * transl(0, 0, -backoff_mm)

    # Makes sure the poses below are read in the robot base frame with the master tool, as the assignment requires
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # Lines up with the lever, touches it, slides it, then presses it down to lock the scale
    UR5.MoveJ(solve_joints_near(UR5, approach_pose, LEVER_CONFIG_HINT), True)
    UR5.MoveL(touch_pose, True)
    UR5.MoveL(slide_pose, True)
    UR5.MoveL(press_pose, True)
    run_visual(RDK, "Show_Rancilio_Scale_Lock")

    # Backs straight off the lever and returns the mazzer tool to the tool stand
    UR5.MoveL(retreat_pose, True)
    tls.mazzer_tool_detach_r_ati()


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)

    # Resets the simulation, then shows what parts l and n leave behind, the rancilio tool in the group head and
    # the cup on the scale pan. Simulation only, comment these out on the real robot
    run_visual(RDK, "Reset_Simulation_R")
    run_visual(RDK, "Show_Rancilio_Rancilio_Tool_Rotated")
    run_visual(RDK, "Show_Rancilio_Scale_Cup")

    # Part q carries on from where parts o and p end, holding the mazzer tool, so all three run back to back
    from _partO import run_task_o
    from _partP import run_task_p
    run_task_o(UR5, tls, RDK)
    run_task_p(UR5, tls, RDK)
    run_task_q(UR5, tls, RDK)
