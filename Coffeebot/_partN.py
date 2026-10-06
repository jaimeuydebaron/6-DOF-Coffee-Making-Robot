# _partN.py
# Task n, use the cup tool to pick up the dispensed cup and place it on the rancilio scale pan
# The cup tool holds a cup in two u shaped plates, the lower one moves. Opening drops the lower plate 51 mm, shutting
# lifts it back up under the upper plate. To pick, the open lower plate slides around the cup just above the cup
# dispenser's fins, and shutting lifts the cup. To place, opening lowers the cup onto the pan and the tool backs out.
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, solve_frame_rotation, run_visual, enable_step_mode
from _partM import compute_cup_dispenser_pose


# How far below the shut holder face (point 49) the base of a held cup sits. Shutting lifts the cup until its rim
# presses up under the upper plate, so this does not depend on where the cup was gripped. Taken from the station's
# held cup model, Cup_(UR5), whose tool point is the base of the cup
HELD_CUP_BASE_BELOW_HOLDER_MM = 74.55


def compute_rancilio_scale_pose(fixture_fine_tune_mm):
    # Gets the rancilio scale's origin and a reference point, in both world and local coordinates
    origin_w = get_point(12)
    origin_l = get_point(39)
    ref_w = get_point(13)
    ref_l = get_point(40)

    # Solves for how far the rancilio scale is rotated relative to the world frame
    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    # Builds the rancilio scale's pose in the robot's world frame
    rancilio_scale_pose = transl(*origin_w) * rotz(theta)

    # Applies the fixture calibration correction
    rancilio_scale_pose = rancilio_scale_pose * transl(*fixture_fine_tune_mm)

    return rancilio_scale_pose


def compute_cup_tool_offset(tool_point_local):
    # Sets the cup tool's contact point and its 50 degree mounting angle offset from the master tool,
    # the same tool adapter offset as the rancilio tool in parts a and l
    return rotz(-50 * pi/180) * transl(*tool_point_local)


def compute_task_n_pick_pose(fixture_fine_tune_mm, grip_below_rim_mm, approach_yaw_deg, offset_mm=(0, 0, 0)):
    cup_dispenser_pose = compute_cup_dispenser_pose()
    cup_dispenser_pose = cup_dispenser_pose * transl(*fixture_fine_tune_mm)

    # Sets the placement point as the dispensed cup's rim centre, with the tool orientation found by inspection,
    # the cup dispenser's x axis already points up and its z axis points in from the front of the dispenser,
    # so the cup tool's axes line up with the fixture's axes one to one
    point_local = get_point(20)
    axis_mapping = [
        [1, 0, 0],  # Tool x axis lines up with the fixture's x axis, straight up
        [0, 1, 0],  # Tool y axis lines up with the fixture's y axis
        [0, 0, 1],  # Tool z axis lines up with the fixture's z axis, the fork slides in away from the robot
    ]
    T_fixture_orientation = build_pose(axis_mapping, (0, 0, 0))

    # Spins the tool about the cup's vertical axis by approach_yaw_deg, the fixture's x axis is the vertical here
    T_fixture_placement = transl(*point_local) * rotx(approach_yaw_deg * pi/180) * T_fixture_orientation

    # Shifts the target along the tool's own axes by offset_mm, x is up and z is the way the fork slides in,
    # zero for the grip pose itself
    T_world_target = cup_dispenser_pose * T_fixture_placement * transl(*offset_mm)

    # Sets the cup tool's contact point as the open holder face (point 48), raised by grip_below_rim_mm, so the
    # rim sits that far above the open lower plate
    holder_open = get_point(48)
    tool_point_local = (holder_open[0] + grip_below_rim_mm, holder_open[1], holder_open[2])
    T_tool_offset = compute_cup_tool_offset(tool_point_local)
    task_n_pick_pose = T_world_target * T_tool_offset.inv()

    return task_n_pick_pose


def compute_task_n_place_pose(fixture_fine_tune_mm, release_height_mm, approach_yaw_deg, offset_mm=(0, 0, 0)):
    rancilio_scale_pose = compute_rancilio_scale_pose(fixture_fine_tune_mm)

    # Sets the placement point as the pan centre, lifted by release_height_mm so the cup is let go above it,
    # with the tool orientation found by inspection, upright with the fork sliding in along the scale's x axis,
    # then spun about the vertical by approach_yaw_deg so the tool can stay clear of the rancilio tool
    point_local = get_point(41)
    axis_mapping = [
        [0, 0, 1],   # Tool x axis lines up with the fixture's z axis, straight up
        [0, -1, 0],  # Tool y axis lines up with the fixture's negative y axis
        [1, 0, 0],   # Tool z axis lines up with the fixture's x axis, the fork slides out over the pan
    ]
    T_fixture_orientation = build_pose(axis_mapping, (0, 0, 0))
    T_fixture_placement = transl(*point_local) * transl(0, 0, release_height_mm) \
                        * rotz(approach_yaw_deg * pi/180) * T_fixture_orientation

    # Shifts the target along the tool's own axes by offset_mm, x is up and z is the way the fork slides in,
    # zero for the release pose itself
    T_world_target = rancilio_scale_pose * T_fixture_placement * transl(*offset_mm)

    # Sets the cup tool's contact point as the base of the held cup, below the shut holder face (point 49)
    holder_shut = get_point(49)
    tool_point_local = (holder_shut[0] - HELD_CUP_BASE_BELOW_HOLDER_MM, holder_shut[1], holder_shut[2])
    T_tool_offset = compute_cup_tool_offset(tool_point_local)
    task_n_place_pose = T_world_target * T_tool_offset.inv()

    return task_n_place_pose


# Jogged joint waypoints, left hand with the elbow and the wrist up (J5 positive), the same way round as tasks l and
# m, the wrist down solution puts the arm on the table cushion at the cup dispenser
# Facing the cup dispenser up high, backed off from the cup, where the straight move down in front of the cup starts
pick_approach_intermediate_pose_1 = [11.118106, -113.551289, 128.837718, -15.286429, 100.655278, 140.000000]
pick_approach_intermediate_pose_2 = [47.845866, -121.987468, 125.238905, -93.251254, -89.998664, -277.153989]
# Facing the coffee machine at the carry height, backed off from the pan, where the straight move down to the pan
# starts
place_approach_intermediate_pose_1 = [-11.332829, -109.391009, 126.657459, -17.266450, 19.055373, 140.000000]


def run_task_n(UR5, tls, RDK):
    """ Does not return the robot home, ends empty handed in the left hand configuration at the cup tool's
    approach pose on the tool stand, with the cup standing on the rancilio scale pan """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # ---------- Picking the cup out of the cup dispenser ----------

    # pick_fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the tool grips the cup, by shifting the assumed position of the cup dispenser along the
    # dispenser's own axes. Every pick pose moves with it.
    #   x: raises (+) or lowers (-) the grip, the dispenser's x points up
    #   y: shifts along the dispenser's local y
    #   z: shifts along the dispenser's local z, the way the fork slides in
    pick_fixture_fine_tune_mm = (0, 0, 0)

    # pick_yaw_deg
    # Turns the way the fork slides in, about the cup's vertical axis, in degrees. 0 is straight in, square to
    # the cups.
    pick_yaw_deg = 0

    # grip_below_rim_mm
    # How far under the rim the open lower plate slides in. Keep it above the dispenser's fins.
    grip_below_rim_mm = 28

    # pick_backoff_mm
    # How far back the fork lines up before sliding in around the cup, and backs out to after gripping it.
    pick_backoff_mm = 120

    # pick_lift_mm
    # Extra lift once the cup is gripped, before backing out. Enough to clear the fins, but not into the cup stack
    # above.
    pick_lift_mm = 20

    # ---------- Placing the cup on the rancilio scale pan ----------

    # place_fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the cup is let go, by shifting the assumed position of the rancilio scale along the scale's own
    # axes. Every place pose moves with it.
    #   x: shifts along the scale's local x
    #   y: shifts along the scale's local y
    #   z: raises (+) or lowers (-) the release
    place_fixture_fine_tune_mm = (0, 0, 0)

    # place_yaw_deg
    # Turns the way the fork slides in, about the vertical, in degrees. 0 is straight into the coffee machine.
    place_yaw_deg = 0

    # release_height_mm
    # How high the cup's base is above the pan when the holder opens.
    release_height_mm = 5

    # place_backoff_mm
    # How far back the fork lines up before sliding in over the pan, and backs out to after letting go.
    place_backoff_mm = 140

    # ---------- Getting between places ----------

    # stand_pull_out_mm
    # How far straight out of the tool stand the tool comes, along the stand's own x, before turning.
    stand_pull_out_mm = 150

    # carry_height_mm
    # How high above the lined up poses the cup is carried across between the dispenser and the scale.
    carry_height_mm = 80

    # holder_wait_s
    # How long to wait after opening or shutting the holder, so it finishes moving.
    holder_wait_s = 1.0

    # The joint poses above (pick_approach_intermediate_pose_1 and place_approach_intermediate_pose_1) are jogged
    # joint angles, not affected by the settings here. They are where the straight moves in to the cup dispenser and
    # the scale start from, so if you move a fixture or change a yaw or a backoff a lot, re-jog them.
    # =============================================================================

    # Computes the poses at the cup dispenser, lined up high, lined up level, gripping, lifted, and backed out.
    # Offsets are in the tool's own axes, x is up and z is the way the fork slides in
    pick_above_pose = compute_task_n_pick_pose(
        pick_fixture_fine_tune_mm, grip_below_rim_mm, pick_yaw_deg, (carry_height_mm, 0, -pick_backoff_mm))
    pick_approach_pose = compute_task_n_pick_pose(
        pick_fixture_fine_tune_mm, grip_below_rim_mm, pick_yaw_deg, (0, 0, -pick_backoff_mm))
    pick_pose = compute_task_n_pick_pose(
        pick_fixture_fine_tune_mm, grip_below_rim_mm, pick_yaw_deg)
    pick_lift_pose = compute_task_n_pick_pose(
        pick_fixture_fine_tune_mm, grip_below_rim_mm, pick_yaw_deg, (pick_lift_mm, 0, 0))
    pick_retreat_pose = compute_task_n_pick_pose(
        pick_fixture_fine_tune_mm, grip_below_rim_mm, pick_yaw_deg, (pick_lift_mm, 0, -pick_backoff_mm))

    # Computes the poses at the rancilio scale, lined up high, lined up level, and releasing
    place_above_pose = compute_task_n_place_pose(
        place_fixture_fine_tune_mm, release_height_mm, place_yaw_deg, (carry_height_mm, 0, -place_backoff_mm))
    place_approach_pose = compute_task_n_place_pose(
        place_fixture_fine_tune_mm, release_height_mm, place_yaw_deg, (0, 0, -place_backoff_mm))
    place_pose = compute_task_n_place_pose(
        place_fixture_fine_tune_mm, release_height_mm, place_yaw_deg)

    # Makes sure the poses below are read in the robot base frame with the master tool, as the assignment requires
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # Picks up the cup tool, then pulls straight out of the tool stand, along the stand's own x axis, before
    # turning, so the tool can not swing into the stand
    tls.cup_tool_attach_l_ati()
    UR5.MoveJ(pick_approach_intermediate_pose_2, True)

    # ---------- Picks the cup up out of the cup dispenser ----------

    # Turns to face the cup dispenser up high at the jogged pose, comes down in front of the cup, and opens the holder
    UR5.MoveJ(pick_approach_intermediate_pose_1, True)
    UR5.MoveL(pick_above_pose, True)
    UR5.MoveL(pick_approach_pose, True)
    tls.cup_tool_open_ur5()
    sleep(holder_wait_s)

    # Slides the fork in around the cup, and shuts the holder to lift the cup
    UR5.MoveL(pick_pose, True)
    tls.cup_tool_shut_ur5()
    sleep(holder_wait_s)
    run_visual(RDK, "Hide_Cup_Dispenser_Cup")
    run_visual(RDK, "Show_Cup_Tool_Cup")

    # Lifts the cup clear of the fins, backs out of the cup dispenser, and rises to the carry height
    UR5.MoveL(pick_lift_pose, True)
    UR5.MoveL(pick_retreat_pose, True)
    UR5.MoveL(pick_above_pose, True)

    # ---------- Places the cup on the rancilio scale pan ----------

    # Carries the cup straight across up high to the jogged pose, turning at the base and the wrist to face the
    # coffee machine, with no pulling back, then comes down in line with the pan and slides the cup in over it. A
    # joint move, since a straight line here passes near a wrist singularity
    UR5.MoveJ(place_approach_intermediate_pose_1, True)
    UR5.MoveL(place_approach_pose, True)
    UR5.MoveL(place_pose, True)

    # Opens the holder, the lower plate drops and lowers the cup onto the pan
    tls.cup_tool_open_ur5()
    sleep(holder_wait_s)
    run_visual(RDK, "Hide_Cup_Tool_Cup")
    run_visual(RDK, "Show_Rancilio_Scale_Cup")

    # Backs the fork out from around the cup, rises, and shuts the holder again
    UR5.MoveL(place_approach_pose, True)
    UR5.MoveL(place_above_pose, True)
    tls.cup_tool_shut_ur5()

    # Goes straight back to the point out from the tool stand, and returns the cup tool
    UR5.MoveJ(pick_approach_intermediate_pose_2, True)
    tls.cup_tool_detach_l_ati()


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

    # run_visual(RDK, "Reset_Simulation_L")

    # Shows the rancilio tool set down on the mazzer scale and takes it off the tool stand, where task l leaves it
    # while tasks m and n run, and the cup sitting in the cup dispenser, dropped there by task m. Simulation only
    run_visual(RDK, "Show_Mazzer_Scale_Rancilio_Tool")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)
    run_visual(RDK, "Show_Cup_Dispenser_Cup")

    run_task_n(UR5, tls, RDK)

    # Sends the robot home, in the full run task l carries straight on from the tool stand
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
