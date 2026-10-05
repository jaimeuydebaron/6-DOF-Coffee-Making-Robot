# _partF.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _partA import tilt_tool_pose
from _common import compute_waypoints, run_visual, enable_step_mode


def compute_task_f_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg=0, hover_mm=0):
    # Reuses part A's tilted placement pose, so the tool is grabbed at exactly the position and angle
    # part A left it at. Pass the same values part A used. hover_mm lifts it straight up along the
    # scale's z, keeping the tilt, the same as part A's align pose.
    return tilt_tool_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm, extra_tilt_deg)


# Jogged waypoints for approaching the tool sitting on the mazzer scale, and
# for retreating clear of the mazzer body afterward
approach_intermediate_pose_1 = [118.506782, -100.274943, 101.666651, -91.307901, -89.525705, -206.644186]
approach_intermediate_pose_2 = [147.869587, -102.441040, 117.125449, -15.935730, 119.139464, 139.402718]
rancilio_align_intermediate_pose = [147.869438, -79.274625, 137.134086, -59.110450, 119.138938, 139.403203]

retreat_intermediate_pose_1 = [147.869438, -84.061570, 135.628719, -52.818138, 119.138938, 139.403203]


def run_task_f(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================
    # This pose is where the robot grabs the rancilio tool back off the ball bearing, so it should match
    # where and how part A left it.

    # fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the robot goes to grab the tool, by shifting the assumed position of the mazzer scale
    # along the scale's own axes. The tool keeps its angle and just moves as a whole.
    #   x: shifts along the scale's local x
    #   y: shifts along the scale's local y
    #   z: raises (+) or lowers (-) the grab pose, use this if the tool is grabbed too high or too low
    # MUST equal part A's fixture_fine_tune_mm, or the grab pose will not be where part A left the tool.
    fixture_fine_tune_mm = (0, 0, -4)

    # approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z), applied before the extra tilt.
    # MUST equal part A's approach_angle_adjust_deg. It pivots about the tool's origin, NOT the ball bearing
    # point, so changing it also moves where the tip lands.
    approach_angle_adjust_deg = (0, 0, 0)

    # extra_tilt_deg
    # Extra lean added on top of the 1.7 degree stem angle (point 63), about the tool's own y, pivoting about
    # point 54. MUST equal part A's extra_tilt_deg, so the tool is grabbed at the same angle it was placed at.
    extra_tilt_deg = 0

    # align_hover_mm
    # How far straight up (along the scale's z) the tool lifts, keeping its tilt, right after it is grabbed,
    # before heading to the retreat poses. This is part A's approach run backwards.
    # Bigger = lifts higher before moving off, smaller = lifts less. It does not change the grab pose.
    # Keep it the same as part A's align_hover_mm so the lift retraces the way part A dropped the tool in.
    align_hover_mm = 30

    # The pose is built by part A's tilt_tool_pose with no hover, so with matching values above it is identical
    # to part A's final placement pose.
    # The jogged joint poses (approach_intermediate_pose_1/2, rancilio_align_intermediate_pose,
    # retreat_intermediate_pose_1) are NOT affected by the settings above. If you move the grab pose a lot,
    # re-jog rancilio_align_intermediate_pose so the straight move in to the tool stays clear.
    # =============================================================================

    # Computes the final grab pose, and the pose hovering straight above it that the tool lifts to afterward
    task_f_pose = compute_task_f_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg)
    task_f_hover_pose = compute_task_f_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, extra_tilt_deg, align_hover_mm)

    # Optional, builds extra waypoints from the final tilted pose instead of using align_hover_mm
    # Each offset is (name, (x, y, z) in mm, (rx, ry, rz) in degrees), applied in the tool's OWN frame,
    # translation first then rotation. Each waypoint comes from task_a_pose on its own, they do not chain.
    # For example, this backs the tool off 40 mm along its own z, keeping the tilt:
    waypoints = compute_waypoints(task_f_hover_pose, offsets=[
        ("backoff", (0, 0, -80), (0, 0, 0)),
    ])
    # UR5.MoveL(waypoints["backoff"], True)
    # UR5.MoveL(task_a_pose, True)

    # Moves in to where the rancilio tool is sitting on the scale
    UR5.MoveJ(approach_intermediate_pose_1, True)
    UR5.MoveJ(approach_intermediate_pose_2, True)
    UR5.MoveL(rancilio_align_intermediate_pose, True)
    UR5.MoveL(task_f_pose, True)

    # Grabs the tool, hides the ghost object, and shows it attached to the robot instead
    attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
    attached_tool.setVisible(True, False)
    run_visual(RDK, "Hide_Mazzer_Scale_Rancilio_Tool")

    tls.student_tool_attach()

    # Lifts straight up to the hover pose, keeping the tilt, now holding the tool
    UR5.MoveL(task_f_hover_pose, True)
    UR5.MoveL(retreat_intermediate_pose_1, True)


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)

    # Waits for enter before every move, for testing this task on its own, comment out to run straight through
    enable_step_mode(UR5, tls)

    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # Shows the rancilio tool sitting on the mazzer scale and takes it off the tool stand, as task a leaves it.
    # Simulation only
    run_visual(RDK, "Show_Mazzer_Scale_Rancilio_Tool")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    run_task_f(UR5, tls, RDK)

    # Returns the rancilio tool and sends the robot home, in the full run task g carries on still holding it
    tls.rancilio_tool_detach_l_ati()
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
