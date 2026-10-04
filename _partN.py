# _partN.py
# Task n, use the cup tool to pick up the dispensed cup and place it on the rancilio scale pan
# The cup tool holds a cup in two u shaped plates, the lower one moves. Opening drops the lower plate 51 mm, shutting
# lifts it back up under the upper plate. To pick, the open lower plate slides around the cup just above the cup
# dispenser's fins, and shutting lifts the cup. To place, opening lowers the cup onto the pan and the tool backs out.
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, solve_frame_rotation
from _partM import compute_cup_dispenser_pose


# How far below the shut holder face (point 49) the base of a held cup sits. Shutting lifts the cup until its rim
# presses up under the upper plate, so this does not depend on where the cup was gripped. Taken from the station's
# held cup model, Cup_(UR5), whose tool point is the base of the cup
HELD_CUP_BASE_BELOW_HOLDER_MM = 74.55

# Arm configuration hints, right hand with the elbow up. The joint moves into each area go to the calculated pose,
# solved for the joint angles closest to these, so the arm always arrives the same way round. They only pick which
# of the arm's solutions is used, they do not change where the tool goes. The two are nearly the same, so carrying
# the cup across is mostly the base turning, and the wrist (J5) is on the side that tilts the tool straight back
# to the tool stand afterwards, about 100 degrees, rather than turning it the long way round
PICK_CONFIG_HINT = [-196, -105, -103, -152, 194, -220]
PLACE_CONFIG_HINT = [-167, -103, -96, -161, 193, -220]


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


def solve_joints_near(UR5, pose, config_hint):
    # Solves the joint angles that put the master tool at pose, choosing the solution closest to config_hint
    joints = UR5.SolveIK(pose, config_hint, UR5.PoseTool()).list()
    if len(joints) < 6:
        raise ValueError("No joint solution for this pose, it may be out of reach")
    return joints[:6]


def run_visual(RDK, program_name):
    # Runs one of the station's visual programs, these only change the simulation
    program = RDK.Item(program_name, ITEM_TYPE_PROGRAM)
    program.RunCode()
    program.WaitFinished()


def run_task_n(UR5, tls, RDK):
    """ Does not return the robot home, ends empty handed in the right hand configuration at the cup tool's
    approach pose on the tool stand, with the cup standing on the rancilio scale pan """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # ----- Picking the cup out of the cup dispenser -----
    pick_fixture_fine_tune_mm = (0, 0, 0)  # moves the grip, along the dispenser's axes, x is up, z is into the dispenser
    pick_yaw_deg = 60                      # turns the way the fork slides in, 60 faces it the same way as at the pan
    grip_below_rim_mm = 28                 # how far under the rim the open lower plate slides in, keep it above the fins
    pick_backoff_mm = 120                  # how far back the fork lines up before sliding in, and backs out to
    pick_lift_mm = 20                      # extra lift before backing out, clears the fins but not into the cup stack above

    # ----- Placing the cup on the rancilio scale pan -----
    place_fixture_fine_tune_mm = (0, 0, 0)  # moves the release, along the scale's axes, z is up
    place_yaw_deg = 30                      # turns the way the fork slides in, 25 hits the rancilio tool, 35 the dispenser
    release_height_mm = 5                   # how high the cup's base is above the pan when the holder opens
    place_backoff_mm = 140                  # how far back the fork lines up before sliding in, and backs out to

    # ----- Getting between places -----
    stand_pull_out_mm = 150  # how far straight out of the tool stand the tool comes before turning
    pick_entry_raise_mm = 60  # comes in this much higher over the dispenser after leaving the stand, then straight down
    carry_height_mm = 80     # how high above the lined up poses the cup is carried across between the two
    carry_via_mm = (120, 120, 0)  # sidestep on the way across, world mm from above the pan, keeps the cup off the
                                  # dispenser, +x is towards the mazzer side, +y towards the customer zone
    holder_wait_s = 1.0      # how long to wait after opening or shutting the holder, so it finishes moving
    # =============================================================================

    # Computes the poses at the cup dispenser, lined up high, lined up level, gripping, lifted, and backed out.
    # Offsets are in the tool's own axes, x is up and z is the way the fork slides in
    def pick(offset_mm):
        return compute_task_n_pick_pose(pick_fixture_fine_tune_mm, grip_below_rim_mm, pick_yaw_deg, offset_mm)

    pick_entry_pose = pick((carry_height_mm + pick_entry_raise_mm, 0, -pick_backoff_mm))
    pick_above_pose = pick((carry_height_mm, 0, -pick_backoff_mm))
    pick_approach_pose = pick((0, 0, -pick_backoff_mm))
    pick_pose = pick((0, 0, 0))
    pick_lift_pose = pick((pick_lift_mm, 0, 0))
    pick_retreat_pose = pick((pick_lift_mm, 0, -pick_backoff_mm))

    # Computes the poses at the rancilio scale, lined up high, lined up level, and releasing
    def place(offset_mm):
        return compute_task_n_place_pose(place_fixture_fine_tune_mm, release_height_mm, place_yaw_deg, offset_mm)

    place_above_pose = place((carry_height_mm, 0, -place_backoff_mm))
    place_approach_pose = place((0, 0, -place_backoff_mm))
    place_pose = place((0, 0, 0))

    # The sidestep point on the way across, the same tool angle as above the pan
    carry_via_pose = transl(*carry_via_mm) * place_above_pose

    # Makes sure the poses below are read in the robot base frame with the master tool, as the assignment requires
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # Shows the cup sitting in the cup dispenser, dropped there by task m
    run_visual(RDK, "Show_Cup_Dispenser_Cup")

    # Picks up the cup tool, then pulls straight out of the tool stand, along the stand's own x axis, before
    # turning, so the tool can not swing into the stand
    tls.cup_tool_attach_r_ati()
    stand_frame = RDK.Item("Cup_Tool_(ATI)", ITEM_TYPE_FRAME).PoseAbs()
    pull_out = [stand_frame[i, 0] * stand_pull_out_mm for i in range(3)]
    stand_clear_pose = transl(*pull_out) * UR5.Pose()
    UR5.MoveL(stand_clear_pose, True)
    stand_clear_joints = UR5.Joints()

    # ---------- Picks the cup up out of the cup dispenser ----------

    # Turns to face the cup dispenser up high, comes down in front of the cup, and opens the holder
    UR5.MoveJ(solve_joints_near(UR5, pick_entry_pose, PICK_CONFIG_HINT), True)
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

    # Carries the cup across up high, sidestepping away from the dispenser, comes down in line with the pan, then
    # slides the cup in over it. The cup faces almost the same way at both ends, so this is mostly the base
    # turning. Joint moves, since a straight line here passes near a wrist singularity
    UR5.MoveJ(solve_joints_near(UR5, carry_via_pose, PLACE_CONFIG_HINT), True)
    UR5.MoveJ(solve_joints_near(UR5, place_above_pose, PLACE_CONFIG_HINT), True)
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
    UR5.MoveJ(stand_clear_joints, True)
    tls.cup_tool_detach_r_ati()


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)

    # Resets the simulation, then shows the rancilio tool locked in the group head, left there by task l, so a
    # standalone test checks the place against it. Simulation only, comment both out on the real robot
    run_visual(RDK, "Reset_Simulation_R")
    run_visual(RDK, "Show_Rancilio_Rancilio_Tool_Rotated")

    run_task_n(UR5, tls, RDK)

    # Sends the robot home
    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)
