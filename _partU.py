# _partU.py
# Task u, position the rancilio tool over the rancilio tool cleaner's bristle brush, and actuate it for 5 s
# This is part t on the other brush, it slides straight across from above the silicone brush, then cleans the same way
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import run_visual
from _partT import compute_task_t_pose, clean_on_brush, BRISTLE_BRUSH


def compute_task_u_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm=0):
    # Reuses part t's brush pose on the bristle brush, so the basket meets it the same way it met the silicone brush.
    # Pass the same values part t used
    return compute_task_t_pose(BRISTLE_BRUSH, fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm)


def run_task_u(UR5, tls, RDK):
    """ Does not return the robot home, starts above the silicone brush where task t finishes, and ends holding the
    rancilio tool above the bristle brush, ready to return it to the tool stand in task v """

    # ==================== FINE TUNING (edit these in the lab) ====================
    # These MUST equal the values in run_task_t in _partT.py, so the basket meets the bristle brush the same way it
    # met the silicone brush. See there for what each one does.

    fixture_fine_tune_mm = (0, 0, 0)
    approach_angle_adjust_deg = (0, 0, 0)
    approach_height_mm = 100
    press_mm = 20
    clean_time_s = 5
    # =============================================================================

    # Computes the poses above the bristle brush, on it, and pressing it down
    hover_pose = compute_task_u_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, approach_height_mm)
    brush_pose = compute_task_u_pose(fixture_fine_tune_mm, approach_angle_adjust_deg)
    pressed_pose = compute_task_u_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, -press_mm)

    # Slides straight across at the travel height to line the basket up above the bristle brush
    UR5.MoveL(hover_pose, True)

    # Lowers onto the bristle brush, presses to actuate the cleaner, then lifts clear
    clean_on_brush(UR5, brush_pose, pressed_pose, hover_pose, clean_time_s)

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

    # Puts the rancilio tool in the group head and takes it off the tool stand, as it would be after task l.
    # Simulation only
    run_visual(RDK, "Show_Rancilio_Rancilio_Tool_Rotated")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    run_task_u(UR5, tls, RDK)
