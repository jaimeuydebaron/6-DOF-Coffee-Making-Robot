# _partR.py
# Task r, use the cup tool to carefully pick up the cup of coffee and place it in the customer zone
# The open fork slides onto the cup on the rancilio scale pan, shuts to grip under the rim, then the arm tucks in
# towards the base, turns round at the base, and lowers the cup straight down onto the table in the customer zone
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import math
import tools
from _common import get_point, build_pose, compute_waypoints, run_visual
from _partN import compute_rancilio_scale_pose, compute_cup_tool_offset


# Height of the cup tool's shut holder face above the base of the cup when gripped, from the station's cup model,
# Cup_(UR5) sits 128.05 below the cup tool frame and the holder face is 53.5 below it
CUP_GRIP_HEIGHT_MM = 128.05 - 53.5

# Tool orientation found by inspection, the same at the pan and at the customer zone
CUP_AXIS_MAPPING = [
    [0, 0, 1],   # Tool x axis lines up with the fixture's z axis, straight up, so the holder plate is level
    [0, -1, 0],  # Tool y axis lines up with the fixture's negative y axis
    [1, 0, 0],   # Tool z axis lines up with the fixture's x axis, the way the fork slides onto the cup
]


def compute_customer_zone_pose(fixture_fine_tune_mm):
    # The customer zone is a single point on the table, so it has no orientation of its own, its frame is taken
    # as the world frame moved to that point
    origin_w = get_point(62)
    customer_zone_pose = transl(*origin_w)

    # Applies the fixture calibration correction
    customer_zone_pose = customer_zone_pose * transl(*fixture_fine_tune_mm)

    return customer_zone_pose


def compute_task_r_pick_pose(fixture_fine_tune_mm, approach_yaw_deg):
    rancilio_scale_pose = compute_rancilio_scale_pose(fixture_fine_tune_mm)

    # Sets the placement point as the pan centre raised to the grip height, with the tool orientation found by
    # inspection
    pan_local = get_point(41)
    point_local = (pan_local[0], pan_local[1], pan_local[2] + CUP_GRIP_HEIGHT_MM)
    T_fixture_placement = build_pose(CUP_AXIS_MAPPING, point_local)

    # Swings the approach direction about the cup's vertical axis, so the fork stays centred on the cup
    T_fixture_placement = T_fixture_placement * rotx(approach_yaw_deg * pi/180)
    T_world_target = rancilio_scale_pose * T_fixture_placement

    # Sets the cup tool's contact point, the shut holder face (point 49), and its mounting angle offset
    T_tool_offset = compute_cup_tool_offset(get_point(49))
    task_r_pick_pose = T_world_target * T_tool_offset.inv()

    return task_r_pick_pose


def compute_task_r_place_pose(fixture_fine_tune_mm, approach_yaw_deg):
    customer_zone_pose = compute_customer_zone_pose(fixture_fine_tune_mm)

    # Sets the placement point as the grip height above the table, with the tool orientation found by inspection
    point_local = (0, 0, CUP_GRIP_HEIGHT_MM)
    T_fixture_placement = build_pose(CUP_AXIS_MAPPING, point_local)

    # Swings the approach direction about the cup's vertical axis
    T_fixture_placement = T_fixture_placement * rotx(approach_yaw_deg * pi/180)
    T_world_target = customer_zone_pose * T_fixture_placement

    # Sets the cup tool's contact point, the shut holder face (point 49), and its mounting angle offset
    T_tool_offset = compute_cup_tool_offset(get_point(49))
    task_r_place_pose = T_world_target * T_tool_offset.inv()

    return task_r_place_pose


def cup_base_xyz(tcp_pose):
    # Where the base of a gripped cup is in the world, for a given tcp pose
    holder = tcp_pose * compute_cup_tool_offset(get_point(49))
    return holder[0, 3], holder[1, 3], holder[2, 3] - CUP_GRIP_HEIGHT_MM


def fork_direction_xy(tcp_pose):
    # The horizontal direction the fork points in, which is the cup tool's z axis
    holder = tcp_pose * compute_cup_tool_offset(get_point(49))
    return holder[0, 2], holder[1, 2]


def compute_base_swing_deg():
    # The angle the robot base turns to carry the cup from the rancilio scale pan round to the customer zone,
    # found from the polar angle of each point about the robot base
    pan_w = get_point(12)
    zone_w = get_point(62)
    swing = math.atan2(zone_w[1], zone_w[0]) - math.atan2(pan_w[1], pan_w[0])
    return (swing * 180/pi + 180) % 360 - 180


def pull_in_radially(tcp_pose, radius_mm):
    # Slides a pose horizontally along the line to the robot base until the cup is radius_mm from the base,
    # which tucks the arm in before the base is turned
    x, y, _ = cup_base_xyz(tcp_pose)
    r = math.hypot(x, y)
    return transl(x/r * (radius_mm - r), y/r * (radius_mm - r), 0) * tcp_pose


def run_task_r(UR5, tls, RDK):
    """ Does not return the robot home, ends with the cup in the customer zone and the cup tool back on the tool
    stand, in the left hand configuration """

    # ==================== FINE TUNING (edit these in the lab) ====================

    # pick_fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the cup is gripped, by shifting the assumed position of the rancilio scale along the scale's own
    # axes. Every pick pose moves with it.
    #   x: shifts along the scale's local x
    #   y: shifts along the scale's local y
    #   z: raises (+) or lowers (-) the grip
    pick_fixture_fine_tune_mm = (0, 0, 0)

    # place_fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the cup is set down, by shifting the customer zone point along the world's axes.
    # z raises (+) or lowers (-) the cup, use it if the cup is dropped from too high or pressed into the table.
    place_fixture_fine_tune_mm = (0, 0, 2)

    # pick_approach_yaw_deg
    # Swings the fork's approach direction about the cup's vertical axis at the pan, in degrees. Keeps the fork
    # clear of the rancilio tool's handle sticking out of the group head.
    pick_approach_yaw_deg = -50

    # place_approach_yaw_adjust_deg
    # Extra swing of the fork's direction about the cup's vertical axis at the customer zone, in degrees, on top of
    # the pick heading turned round by the base. 0 keeps the arm the same shape as at the pick.
    place_approach_yaw_adjust_deg = 0

    # slide_mm
    # How far back the fork starts before sliding onto the cup, and slides back off it after letting go.
    slide_mm = 100

    # lift_mm
    # How far the cup is lifted straight up off the pan once gripped, before pulling back.
    lift_mm = 20

    # carry_height_mm
    # The world height the cup's base is carried at between the pan and the customer zone.
    carry_height_mm = 110

    # carry_radius_mm
    # How close to the robot base the arm is tucked in, measured to the cup, before the base is turned.
    carry_radius_mm = 380

    # There are no jogged joint poses in this part, every pose is calculated from the settings above. The carry
    # poses are joint moves, see the comments in the motion below.
    # =============================================================================

    # Computes the pose gripping the cup on the pan
    pick_pose = compute_task_r_pick_pose(pick_fixture_fine_tune_mm, pick_approach_yaw_deg)

    # Computes the pose setting the cup down in the customer zone. It is the pick pose turned about the robot base,
    # so the arm keeps the same shape and the carry across is a pure base joint turn with the arm tucked in
    base_swing_deg = compute_base_swing_deg()
    pick_heading_deg = math.atan2(*reversed(fork_direction_xy(pick_pose))) * 180/pi
    place_approach_yaw_deg = pick_heading_deg + base_swing_deg + place_approach_yaw_adjust_deg
    place_pose = compute_task_r_place_pose(place_fixture_fine_tune_mm, place_approach_yaw_deg)

    # The tool's z is the slide direction, so backing off along -z pulls the fork clear of the cup
    pick_waypoints = compute_waypoints(pick_pose, offsets=[
        ("approach", (0, 0, -slide_mm), (0, 0, 0)),
    ])
    place_waypoints = compute_waypoints(place_pose, offsets=[
        ("retreat", (0, 0, -slide_mm), (0, 0, 0)),
    ])

    # The tcp is twisted 50 degrees from the cup tool, so the vertical moves are done in the world frame
    pick_raise_mm = carry_height_mm - cup_base_xyz(pick_pose)[2]
    place_raise_mm = carry_height_mm - cup_base_xyz(place_pose)[2]
    pick_lift_pose = transl(0, 0, lift_mm) * pick_pose
    pick_pull_back_pose = transl(0, 0, lift_mm) * pick_waypoints["approach"]
    pick_raised_pose = transl(0, 0, pick_raise_mm) * pick_waypoints["approach"]

    # The carry poses, tucked in towards the base at the carry height before and after turning round, then above
    # the customer zone
    pick_tucked_pose = pull_in_radially(pick_raised_pose, carry_radius_mm)
    place_tucked_pose = rotz(base_swing_deg * pi/180) * pick_tucked_pose
    place_above_pose = transl(0, 0, place_raise_mm) * place_pose

    # Picks up the cup tool from the tool stand and opens it so the fork sits low on the cup
    tls.cup_tool_attach_l_ati()
    tls.cup_tool_open_ur5()

    # Comes in high and tucked in so the tool clears the rancilio tool cleaner, drops to the approach, then slides
    # the open fork onto the cup and shuts it to grip under the rim
    UR5.MoveJ(pick_tucked_pose, True)
    UR5.MoveJ(pick_raised_pose, True)
    UR5.MoveL(pick_waypoints["approach"], True)
    UR5.MoveL(pick_pose, True)
    tls.cup_tool_shut_ur5()

    # Moves the cup from the scale onto the tool
    run_visual(RDK, "Hide_Rancilio_Scale_Cup")
    run_visual(RDK, "Show_Cup_Tool_Cup")

    # Lifts the cup off the pan, pulls straight back out from under the group head, then raises to the carry height
    UR5.MoveL(pick_lift_pose, True)
    UR5.MoveL(pick_pull_back_pose, True)
    UR5.MoveL(pick_raised_pose, True)

    # Pulls the arm in towards the base, turns round to face the customer zone, then reaches out over the zone and
    # lowers the cup straight down onto the table. These are joint moves because the wrist passes through j5 = 180,
    # where a straight line move is singular, but j2 + j3 + j4 stays constant between the carry poses so the cup is
    # held level the whole way
    UR5.MoveJ(pick_tucked_pose, True)
    UR5.MoveJ(place_tucked_pose, True)
    UR5.MoveJ(place_above_pose, True)
    UR5.MoveL(place_pose, True)

    # Opens the tool so the fork drops away from the rim, leaving the cup on the table
    tls.cup_tool_open_ur5()
    run_visual(RDK, "Hide_Cup_Tool_Cup")

    # Slides the fork back off the cup, rises, and returns the cup tool
    UR5.MoveL(place_waypoints["retreat"], True)
    UR5.MoveL(transl(0, 0, place_raise_mm) * place_waypoints["retreat"], True)
    tls.cup_tool_shut_ur5()
    tls.cup_tool_detach_l_ati()

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

    # Shows the cup on the rancilio scale, as it would be after task q. Simulation only
    run_visual(RDK, "Show_Rancilio_Scale_Cup")

    run_task_r(UR5, tls, RDK)
