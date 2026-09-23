# _partR.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from __points import POINTS
import math


def get_point(key):
    entry = POINTS[key]
    return entry[3], entry[4], entry[5]


def build_pose(rotation_columns, xyz):
    col_x, col_y, col_z = rotation_columns
    pose = eye(4)
    for i in range(3):
        pose[i, 0] = col_x[i]
        pose[i, 1] = col_y[i]
        pose[i, 2] = col_z[i]
    pose[0, 3], pose[1, 3], pose[2, 3] = xyz
    return pose


def compute_waypoints(target_pose, offsets):
    # builds a dict of named waypoints, each derived from target_pose by
    # a translation and rotation offset in the tool's own local frame
    # offsets: list of (name, (x_mm, y_mm, z_mm), (rx_deg, ry_deg, rz_deg))
    waypoints = {}

    for name, translation_mm, angle_deg in offsets:
        angle_offset = rotx(angle_deg[0]*pi/180) \
                     * roty(angle_deg[1]*pi/180) \
                     * rotz(angle_deg[2]*pi/180)
        waypoints[name] = target_pose * transl(*translation_mm) * angle_offset

    return waypoints


# height of the cup tool's shut holder face above the base of the cup when gripped,
# from the station's cup model: Cup_(UR5) sits 128.05 below the cup tool frame, holder face is 53.5 below
CUP_GRIP_HEIGHT_MM = 128.05 - 53.5

# tool x points straight up, so the holder plate is level and the cup hangs upright
# tool z points horizontally along the fixture's x, which is the direction the fork slides onto the cup
# tool y completes the right handed frame, x cross y = z
CUP_AXIS_MAPPING = [[0,0,1], [0,-1,0], [1,0,0]]


def compute_cup_tool_offset():
    # sets the cup tool's holder point (shut) and mounting angle offset, same 50 deg adapter twist as part a
    tool_point_local = get_point(49)
    return rotz(-50 * pi/180) * transl(*tool_point_local)


def compute_task_r_pick_pose(fixture_fine_tune_mm, approach_yaw_deg):
    # gets the rancilio scale origin and reference point in world and local coordinates
    origin_w = get_point(12)
    origin_l = get_point(39)
    ref_w = get_point(13)
    ref_l = get_point(40)

    # finds the displacement between origin and reference point in both frames
    world_dx = ref_w[0] - origin_w[0]
    world_dy = ref_w[1] - origin_w[1]
    local_dx = ref_l[0] - origin_l[0]
    local_dy = ref_l[1] - origin_l[1]

    # solves for the rotation angle between the two frames
    cross = local_dx * world_dy - local_dy * world_dx
    dot   = local_dx * world_dx + local_dy * world_dy
    theta = math.atan2(cross, dot)

    # builds the rancilio scale's pose in the robot's world frame
    rancilio_scale_pose = transl(*origin_w) * rotz(theta)

    # applies the fixture calibration correction
    rancilio_scale_pose = rancilio_scale_pose * transl(*fixture_fine_tune_mm)

    # sets the placement point as the pan centre raised to the grip height, and the tool's orientation there
    pan_local = get_point(41)
    point_local = (pan_local[0], pan_local[1], pan_local[2] + CUP_GRIP_HEIGHT_MM)
    T_fixture_placement = build_pose(CUP_AXIS_MAPPING, point_local)

    # swings the approach direction about the cup's vertical axis, so the fork stays centred on the cup
    T_fixture_placement = T_fixture_placement * rotx(approach_yaw_deg * pi/180)
    T_world_target = rancilio_scale_pose * T_fixture_placement

    # converts the holder point target into the tcp target
    task_r_pick_pose = T_world_target * compute_cup_tool_offset().inv()

    return task_r_pick_pose


def compute_task_r_place_pose(fixture_fine_tune_mm, approach_yaw_deg):
    # the customer zone is a single point on the table, so it has no orientation of its own,
    # its frame is taken as the world frame moved to that point
    origin_w = get_point(62)
    customer_zone_pose = transl(*origin_w)

    # applies the fixture calibration correction
    customer_zone_pose = customer_zone_pose * transl(*fixture_fine_tune_mm)

    # sets the placement point as the grip height above the table, and the tool's orientation there
    point_local = (0, 0, CUP_GRIP_HEIGHT_MM)
    T_fixture_placement = build_pose(CUP_AXIS_MAPPING, point_local)

    # swings the approach direction about the cup's vertical axis
    T_fixture_placement = T_fixture_placement * rotx(approach_yaw_deg * pi/180)
    T_world_target = customer_zone_pose * T_fixture_placement

    # converts the holder point target into the tcp target
    task_r_place_pose = T_world_target * compute_cup_tool_offset().inv()

    return task_r_place_pose


def cup_base_xyz(tcp_pose):
    # where the base of a gripped cup is in the world, for a given tcp pose
    holder = tcp_pose * compute_cup_tool_offset()
    return holder[0, 3], holder[1, 3], holder[2, 3] - CUP_GRIP_HEIGHT_MM


def fork_direction_xy(tcp_pose):
    # horizontal direction the fork points in, which is the cup tool's z axis
    holder = tcp_pose * compute_cup_tool_offset()
    return holder[0, 2], holder[1, 2]


def compute_base_swing_deg():
    # angle the robot base turns to carry the cup from the rancilio scale pan round to the customer zone,
    # found from the polar angle of each point about the robot base
    pan_w = get_point(12)
    zone_w = get_point(62)
    swing = math.atan2(zone_w[1], zone_w[0]) - math.atan2(pan_w[1], pan_w[0])
    return (swing * 180/pi + 180) % 360 - 180


def pull_in_radially(tcp_pose, radius_mm):
    # slides a pose horizontally along the line to the robot base until the cup is radius_mm from the base,
    # which tucks the arm in before the base is turned
    x, y, _ = cup_base_xyz(tcp_pose)
    r = math.hypot(x, y)
    return transl(x/r * (radius_mm - r), y/r * (radius_mm - r), 0) * tcp_pose


def run_task_r(UR5, tls, RDK):
    # edit these in the lab

    # shifts each fixture's whole pose in its own local x, y, z directions to correct calibration drift
    pick_fixture_fine_tune_mm = (0, 0, 0)
    place_fixture_fine_tune_mm = (0, 0, 2)

    # swings the fork's approach direction about the cup's vertical axis, in degrees
    # pick yaw of -75 keeps the fork clear of the rancilio tool handle sticking out of the group head
    pick_approach_yaw_deg = -50
    place_approach_yaw_adjust_deg = 0


    # how far back the fork starts before sliding onto the cup, how far the cup is lifted off the pan,
    # the world height the cup base is carried at, and how close to the base the arm is tucked in to turn
    slide_mm = 100
    lift_mm = 20
    carry_height_mm = 110
    carry_radius_mm = 380

    pick_pose = compute_task_r_pick_pose(pick_fixture_fine_tune_mm, pick_approach_yaw_deg)

    # the place pose is the pick pose turned about the robot base, so the arm keeps the same shape
    # and the carry across is a pure base joint turn with the arm tucked in
    base_swing_deg = compute_base_swing_deg()
    pick_heading_deg = math.atan2(*reversed(fork_direction_xy(pick_pose))) * 180/pi
    place_approach_yaw_deg = pick_heading_deg + base_swing_deg + place_approach_yaw_adjust_deg
    place_pose = compute_task_r_place_pose(place_fixture_fine_tune_mm, place_approach_yaw_deg)

    # tool z is the slide direction, so backing off along -z pulls the fork clear of the cup
    pick_waypoints = compute_waypoints(pick_pose, offsets=[
        ("approach", (0, 0, -slide_mm), (0, 0, 0)),
    ])
    place_waypoints = compute_waypoints(place_pose, offsets=[
        ("retreat", (0, 0, -slide_mm), (0, 0, 0)),
    ])

    # the tcp is twisted 50 deg from the cup tool, so vertical moves are done in the world frame
    pick_raise_mm = carry_height_mm - cup_base_xyz(pick_pose)[2]
    place_raise_mm = carry_height_mm - cup_base_xyz(place_pose)[2]
    pick_lift = transl(0, 0, lift_mm) * pick_pose
    pick_pull_back = transl(0, 0, lift_mm) * pick_waypoints["approach"]
    pick_raised = transl(0, 0, pick_raise_mm) * pick_waypoints["approach"]

    # intermediate positions: tucked in towards the base at carry height, before and after turning round
    pick_tucked = pull_in_radially(pick_raised, carry_radius_mm)
    place_tucked = rotz(base_swing_deg * pi/180) * pick_tucked
    place_above = transl(0, 0, place_raise_mm) * place_pose

    # picks up the cup tool from the tool stand and opens it so the fork sits low on the cup
    tls.cup_tool_attach_l_ati()
    tls.cup_tool_open_ur5()

    # comes in high and tucked in so the tool clears the rancilio tool cleaner, drops to the approach,
    # then slides the open fork onto the cup and shuts it to grip under the rim
    UR5.MoveJ(pick_tucked, True)
    UR5.MoveJ(pick_raised, True)
    UR5.MoveL(pick_waypoints["approach"], True)
    UR5.MoveL(pick_pose, True)
    tls.cup_tool_shut_ur5()

    # moves the cup from the scale onto the tool
    hide_scale_cup = RDK.Item("Hide_Rancilio_Scale_Cup", ITEM_TYPE_PROGRAM)
    hide_scale_cup.RunCode()
    hide_scale_cup.WaitFinished()
    show_tool_cup = RDK.Item("Show_Cup_Tool_Cup", ITEM_TYPE_PROGRAM)
    show_tool_cup.RunCode()
    show_tool_cup.WaitFinished()

    # lifts the cup off the pan, pulls straight back out from under the group head, then raises to carry height
    UR5.MoveL(pick_lift, True)
    UR5.MoveL(pick_pull_back, True)
    UR5.MoveL(pick_raised, True)

    # pulls the arm in towards the base, turns round to face the customer zone,
    # then reaches out over the zone and lowers the cup straight down onto the table
    # these are joint moves because the wrist passes through j5 = 180 (a straight line move is singular there),
    # but j2 + j3 + j4 stays constant between the carry poses so the cup is held level the whole way
    UR5.MoveJ(pick_tucked, True)
    UR5.MoveJ(place_tucked, True)
    UR5.MoveJ(place_above, True)
    UR5.MoveL(place_pose, True)

    # opens the tool so the fork drops away from the rim, leaving the cup on the table
    tls.cup_tool_open_ur5()
    hide_tool_cup = RDK.Item("Hide_Cup_Tool_Cup", ITEM_TYPE_PROGRAM)
    hide_tool_cup.RunCode()
    hide_tool_cup.WaitFinished()

    # slides the fork back off the cup, returns the cup tool and sends the robot home
    UR5.MoveL(place_waypoints["retreat"], True)
    UR5.MoveL(transl(0, 0, place_raise_mm) * place_waypoints["retreat"], True)
    tls.cup_tool_shut_ur5()
    tls.cup_tool_detach_l_ati()
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
    RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # sim only: shows the cup on the rancilio scale, as it would be after task q
    RDK.Item("Show_Rancilio_Scale_Cup", ITEM_TYPE_PROGRAM).RunCode()

    run_task_r(UR5, tls, RDK)
