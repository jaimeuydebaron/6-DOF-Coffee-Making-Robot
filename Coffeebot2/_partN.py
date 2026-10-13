# _partN.py
# Task n, use the cup tool to pick up the dispensed cup and place it on the rancilio scale pan
# The cup tool holds a cup in two u shaped plates, the lower one moves. Opening drops the lower plate 51 mm, shutting
# lifts it back up under the upper plate. To pick, the open lower plate slides around the cup just above the cup
# dispenser's fins, and shutting lifts the cup. To place, opening lowers the cup onto the pan and the tool backs out.
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, solve_frame_rotation, run_visual, enable_space_pause
from _partM import compute_cup_dispenser_pose


# How far below the shut holder face (point 49) the base of a held cup sits. Shutting lifts the cup until its rim
# presses up under the upper plate, so this does not depend on where the cup was gripped. Taken from the station's
# held cup model, Cup_(UR5), whose tool point is the base of the cup
HELD_CUP_BASE_BELOW_HOLDER_MM = 74.55


def compute_rancilio_scale_pose(fixture_fine_tune_mm):
    # The rancilio scale's frame, see part a for how this works
    origin_w = get_point(12)
    origin_l = get_point(39)
    ref_w = get_point(13)
    ref_l = get_point(40)

    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    rancilio_scale_pose = transl(*origin_w) * rotz(theta)

    rancilio_scale_pose = rancilio_scale_pose * transl(*fixture_fine_tune_mm)

    return rancilio_scale_pose


def compute_cup_tool_offset(tool_point_local):
    # The cup tool's contact point, with the same 50 degree mounting offset as the rancilio tool
    return rotz(-50 * pi/180) * transl(*tool_point_local)


def compute_task_n_pick_pose(fixture_fine_tune_mm, grip_below_rim_mm, approach_yaw_deg, offset_mm=(0, 0, 0)):
    cup_dispenser_pose = compute_cup_dispenser_pose()
    cup_dispenser_pose = cup_dispenser_pose * transl(*fixture_fine_tune_mm)

    # The target is the dispensed cup's rim centre (point 20). The dispenser's x points up and its z points in
    # from the front, so the cup tool's axes line up with it one to one
    point_local = get_point(20)
    axis_mapping = [
        [1, 0, 0],  # Tool x axis lines up with the fixture's x axis, straight up
        [0, 1, 0],  # Tool y axis lines up with the fixture's y axis
        [0, 0, 1],  # Tool z axis lines up with the fixture's z axis, the fork slides in away from the robot
    ]
    T_fixture_orientation = build_pose(axis_mapping, (0, 0, 0))

    # Spun about the cup's vertical axis, the fixture's x here, by approach_yaw_deg
    T_fixture_placement = transl(*point_local) * rotx(approach_yaw_deg * pi/180) * T_fixture_orientation

    # Shifted along the tool's own axes by offset_mm, x is up and z is the way the fork slides in
    T_world_target = cup_dispenser_pose * T_fixture_placement * transl(*offset_mm)

    # The contact point is the open holder face (point 48), raised so the rim sits grip_below_rim_mm above the
    # open lower plate
    holder_open = get_point(48)
    tool_point_local = (holder_open[0] + grip_below_rim_mm, holder_open[1], holder_open[2])
    T_tool_offset = compute_cup_tool_offset(tool_point_local)
    task_n_pick_pose = T_world_target * T_tool_offset.inv()

    return task_n_pick_pose


def compute_task_n_place_pose(fixture_fine_tune_mm, release_height_mm, approach_yaw_deg, offset_mm=(0, 0, 0)):
    rancilio_scale_pose = compute_rancilio_scale_pose(fixture_fine_tune_mm)

    # The target is the pan centre (point 41), raised by release_height_mm, with the tool upright and the fork
    # sliding in along the scale's x, then spun about the vertical by approach_yaw_deg
    point_local = get_point(41)
    axis_mapping = [
        [0, 0, 1],   # Tool x axis lines up with the fixture's z axis, straight up
        [0, -1, 0],  # Tool y axis lines up with the fixture's negative y axis
        [1, 0, 0],   # Tool z axis lines up with the fixture's x axis, the fork slides out over the pan
    ]
    T_fixture_orientation = build_pose(axis_mapping, (0, 0, 0))
    T_fixture_placement = transl(*point_local) * transl(0, 0, release_height_mm) \
                        * rotz(approach_yaw_deg * pi/180) * T_fixture_orientation

    # Shifted along the tool's own axes by offset_mm, x is up and z is the way the fork slides in
    T_world_target = rancilio_scale_pose * T_fixture_placement * transl(*offset_mm)

    # The contact point is the base of the held cup, below the shut holder face (point 49)
    holder_shut = get_point(49)
    tool_point_local = (holder_shut[0] - HELD_CUP_BASE_BELOW_HOLDER_MM, holder_shut[1], holder_shut[2])
    T_tool_offset = compute_cup_tool_offset(tool_point_local)
    task_n_place_pose = T_world_target * T_tool_offset.inv()

    return task_n_place_pose


# Jogged joint waypoints, left hand with the elbow and wrist up (J5 positive), the wrist down solution puts the arm
# on the table cushion at the cup dispenser
# Facing the cup dispenser up high, where the straight move down in front of the cup starts
pick_approach_intermediate_pose_1 = [11.118106, -113.551289, 128.837718, -15.286429, 100.655278, -220.000000]
pick_approach_intermediate_pose_2 = [-3.287844, -137.703761, 130.047464, -82.343732, -90.000297, -278.287524]
# Facing the coffee machine at the carry height, where the straight move down to the pan starts
place_approach_intermediate_pose_1 = [-11.332829, -109.391009, 126.657459, -17.266450, 19.055373, -220]


def run_task_n(UR5, tls, RDK):
    """ Does not return the robot home, ends empty handed in the left hand configuration at the cup tool's
    approach pose on the tool stand, with the cup standing on the rancilio scale pan """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # ---------- Picking the cup out of the cup dispenser ----------

    # Shifts where the tool grips the cup, along the dispenser's own axes, x is up and z is the way the fork slides in
    pick_fixture_fine_tune_mm = (0, 0, 0)

    # Turns the way the fork slides in, about the cup's vertical axis. 0 is straight in
    pick_yaw_deg = 0

    # How far under the rim the open lower plate slides in, keep it above the dispenser's fins
    grip_below_rim_mm = 28

    # How far back the fork lines up before sliding in, and backs out to after gripping
    pick_backoff_mm = 120

    # Extra lift once gripped, enough to clear the fins but not into the cup stack above
    pick_lift_mm = 20

    # ---------- Placing the cup on the rancilio scale pan ----------

    # Shifts where the cup is let go, along the rancilio scale's own axes
    place_fixture_fine_tune_mm = (0, 0, 0)

    # Turns the way the fork slides in, about the vertical. 0 is straight into the coffee machine
    place_yaw_deg = 0

    # Height of the cup's base above the pan when the holder opens
    release_height_mm = 5

    # How far back the fork lines up before sliding in over the pan, and backs out to after letting go
    place_backoff_mm = 140

    # ---------- Getting between places ----------

    # Height above the lined up poses that the cup is carried across at
    carry_height_mm = 80

    # Wait after opening or shutting the holder, so it finishes moving
    holder_wait_s = 1.0
    # =============================================================================

    # The poses at the cup dispenser, lined up high, lined up level, gripping, lifted, and backed out
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

    # The poses at the rancilio scale, lined up high, lined up level, and releasing
    place_above_pose = compute_task_n_place_pose(
        place_fixture_fine_tune_mm, release_height_mm, place_yaw_deg, (carry_height_mm, 0, -place_backoff_mm))
    place_approach_pose = compute_task_n_place_pose(
        place_fixture_fine_tune_mm, release_height_mm, place_yaw_deg, (0, 0, -place_backoff_mm))
    place_pose = compute_task_n_place_pose(
        place_fixture_fine_tune_mm, release_height_mm, place_yaw_deg)

    # Reads the poses in the robot base frame with the master tool
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # Picks up the cup tool and moves out from the tool stand
    tls.cup_tool_attach_l_ati()
    UR5.MoveL(pick_approach_intermediate_pose_2, True)

    # ---------- Picks the cup up out of the cup dispenser ----------

    # Turns to face the cup dispenser up high at the jogged pose, comes down in front of the cup, and opens the holder
    UR5.MoveJ(pick_approach_intermediate_pose_1, True)
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

    # Carries the cup across up high to the jogged pose, a joint move since a straight line here passes near a
    # wrist singularity, then comes down in line with the pan and slides the cup in over it
    UR5.MoveL(place_approach_intermediate_pose_1, True)
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

    # Space in the terminal pauses and plays the run, for testing this task on its own
    enable_space_pause(UR5, tls)

    # run_visual(RDK, "Reset_Simulation_L")

    # Shows the rancilio tool set down on the mazzer scale and takes it off the tool stand, where task l leaves it
    # while task n runs, and the cup sitting in the cup dispenser, dropped there by task m. Simulation only
    run_visual(RDK, "Show_Mazzer_Scale_Rancilio_Tool")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)
    run_visual(RDK, "Show_Cup_Dispenser_Cup")

    run_task_n(UR5, tls, RDK)

    # Sends the robot home, in the full run task l carries straight on from the tool stand
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
