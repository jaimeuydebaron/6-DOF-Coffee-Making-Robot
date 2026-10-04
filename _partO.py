# _partO.py
# Task o, use the mazzer tool to unlock the rancilio scale
# The rancilio scale is the same model as the mazzer scale, its lock lever points (42, 43) match points 30 and 31
# exactly, so this unlocks it the same way part b does, the tool tip touches the lever, slides it along the scale's
# -x, then presses down to flip it. The difference is the rancilio tool, locked in the group head by part l, whose
# handle hangs out over the scale, so the tool leans in under it rather than coming straight down
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import get_point, build_pose
from _partN import compute_rancilio_scale_pose, solve_joints_near, run_visual


# Arm configuration hint, right hand with the elbow up. The joint move in goes to the calculated pose, solved for
# the joint angles closest to these, so the arm always arrives the same way round
LEVER_CONFIG_HINT = [-120, -100, -110, -150, 90, -220]


def compute_task_o_pose(fixture_fine_tune_mm, tilt_deg, lean_toward_deg, spin_deg, offset_mm=(0, 0, 0)):
    rancilio_scale_pose = compute_rancilio_scale_pose(fixture_fine_tune_mm)

    # Moves the lever point along the scale's own axes by offset_mm, zero for first contact, then the slide and the
    # press are offsets from there, the same as part b
    rancilio_scale_pose = rancilio_scale_pose * transl(*offset_mm)

    # Sets the placement point as the right lock lever, with the tool orientation found by inspection, the same
    # as part b, the tool pointing straight down at the lever
    point_local = get_point(43)
    axis_mapping = [
        [0, 1, 0],   # Tool x axis lines up with the fixture's y axis
        [1, 0, 0],   # Tool y axis lines up with the fixture's x axis
        [0, 0, -1],  # Tool z axis lines up with the fixture's negative z axis, pointing down at the lever
    ]
    T_straight_down = build_pose(axis_mapping, (0, 0, 0))

    # Leans the tool over by tilt_deg, about the lever point, so its back end (the flange) leans towards
    # lean_toward_deg, measured in the scale's frame from its x axis, 180 leans back towards the robot. Then spins
    # it about its own axis by spin_deg, which only moves the dose bar, the tip stays on the lever
    T_lean = rotz(lean_toward_deg * pi/180) * roty(tilt_deg * pi/180)
    T_fixture_placement = transl(*point_local) * T_lean * T_straight_down * rotz(spin_deg * pi/180)
    T_world_target = rancilio_scale_pose * T_fixture_placement

    # Sets the mazzer tool's contact point, the cushioning cylinder tip (point 51)
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    return T_world_target * T_tool_offset.inv()


def run_task_o(UR5, tls, RDK):
    """ Ends holding the mazzer tool, backed away from the lever, ready for part p """

    # ==================== FINE TUNING (edit these in the lab) ====================
    fixture_fine_tune_mm = (10, 0, 0)  # moves every pose, along the scale's axes, the same start as part b
    tilt_deg = 60                      # how far the tool leans over from straight down, to get under the rancilio tool
    lean_toward_deg = 180              # which way it leans, in the scale's frame, 180 is back towards the robot
    spin_deg = 0                       # spins the tool about its own axis, only moves the dose bar out of the way
    slide_mm = (-35, 0, 0)             # how far the tip slides the lever, along the scale's axes, the same as part b
    press_mm = (0, 0, -10)             # then how far it presses down past the slide, bigger if the lever won't flip
    backoff_mm = 80                    # how far back along the tool the tip lines up before, and backs off to after
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

    # Picks up the mazzer tool from the tool stand
    tls.mazzer_tool_attach_r_ati()

    # Lines up with the lever, touches it, slides it, then presses it down to unlock the scale
    UR5.MoveJ(solve_joints_near(UR5, approach_pose, LEVER_CONFIG_HINT), True)
    UR5.MoveL(touch_pose, True)
    UR5.MoveL(slide_pose, True)
    UR5.MoveL(press_pose, True)
    run_visual(RDK, "Show_Rancilio_Scale_Read")

    # Backs straight off the lever, still holding the mazzer tool for part p
    UR5.MoveL(retreat_pose, True)


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

    run_task_o(UR5, tls, RDK)
