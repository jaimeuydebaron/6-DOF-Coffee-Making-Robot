# _partS.py
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


def compute_task_s_pose(fixture_fine_tune_mm, handle_yaw_deg):
    # gets the rancilio origin and reference point in world and local coordinates
    origin_w = get_point(10)
    origin_l = get_point(35)
    ref_w = get_point(11)
    ref_l = get_point(36)

    # finds the displacement between origin and reference point in both frames
    world_dx = ref_w[0] - origin_w[0]
    world_dy = ref_w[1] - origin_w[1]
    local_dx = ref_l[0] - origin_l[0]
    local_dy = ref_l[1] - origin_l[1]

    # solves for the rotation angle between the two frames
    cross = local_dx * world_dy - local_dy * world_dx
    dot   = local_dx * world_dx + local_dy * world_dy
    theta = math.atan2(cross, dot)

    # builds the rancilio's pose in the robot's world frame
    rancilio_pose = transl(*origin_w) * rotz(theta)

    # applies the fixture calibration correction
    rancilio_pose = rancilio_pose * transl(*fixture_fine_tune_mm)

    # sets the placement point as the group gasket centre, which the basket rim sits against
    # tool x points up so the basket opening faces the gasket, tool z points into the machine (along -x of the rancilio)
    point_local = get_point(38)
    axis_mapping = [[0,0,1], [0,1,0], [-1,0,0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)

    # twists the handle about the group head's vertical axis, 0 is the handle pointing straight out the front
    T_fixture_placement = T_fixture_placement * rotx(handle_yaw_deg * pi/180)
    T_world_target = rancilio_pose * T_fixture_placement

    # sets the rancilio tool's basket rim centre and mounting angle offset
    tool_point_local = get_point(55)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    task_s_pose = T_world_target * T_tool_offset.inv()

    return task_s_pose


# mate approach pose in the right hand configuration, so the joint move from home_r does not flip the elbow
approach_general_point = [-120.1, -96.6, -121.6, -141.7, -75.1, 140.0]

approach_point_1 = [23.561060, -88.604205, 127.365805, -38.761601, 68.598596, 140.000000]

def run_task_s(UR5, tls, RDK):
    # edit these in the lab

    # shifts the rancilio's whole pose in its own local x, y, z directions to correct calibration drift
    fixture_fine_tune_mm = (0, 0, 0)

    # handle twist when locked in the group head, and how far it turns to unlock
    # from the station model: locked handle points back at the robot (-15), inserted position is 30 deg further round
    locked_yaw_deg = -15
    unlock_turn_deg = -30

    # how far back the robot lines up before mating with the tool, how far the basket drops
    # out of the group head (basket depth is 29.3), and how far the tool is pulled out afterwards
    mate_back_off_mm = 60
    drop_mm = 50
    pull_out_mm = 100

    locked_pose = compute_task_s_pose(fixture_fine_tune_mm, locked_yaw_deg)
    unlocked_pose = compute_task_s_pose(fixture_fine_tune_mm, locked_yaw_deg + unlock_turn_deg)

    # tool z runs along the handle, so backing off along -z lines the robot up with the tool's coupling
    waypoints = compute_waypoints(locked_pose, offsets=[
        ("mate_approach", (0, 0, -50), (0, 0, 0)),
    ])

    UR5.MoveJ(approach_point_1, True)

    # drops straight down in the world frame, then pulls back along the handle
    lowered_pose = transl(0, 0, -drop_mm) * unlocked_pose
    pulled_out_pose = lowered_pose * transl(0, 0, -pull_out_mm)

    # # lines up behind the tool in the group head and mates with it
    # UR5.MoveJ(approach_general_point, True)
    # UR5.MoveL(waypoints["mate_approach"], True)
    UR5.MoveL(locked_pose, True)
    tls.student_tool_attach()

    # moves the rancilio tool from the group head onto the robot
    hide_group_head_tool = RDK.Item("Hide_Rancilio_Rancilio_Tool_Rotated", ITEM_TYPE_PROGRAM)
    hide_group_head_tool.RunCode()
    hide_group_head_tool.WaitFinished()
    attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
    attached_tool.setVisible(True, False)

    # twists the handle to unlock, in small steps so the basket turns about its own centre
    steps = 3
    for i in range(1, steps + 1):
        UR5.MoveL(compute_task_s_pose(fixture_fine_tune_mm, locked_yaw_deg + unlock_turn_deg * i / steps), True)

    # drops the basket out of the group head and pulls the tool back a bit from under the machine
    UR5.MoveL(lowered_pose, True)
    UR5.MoveL(pulled_out_pose, True)

    # finishes pulled back from the machine, ready to swing straight over to the tool cleaner in task t


if __name__ == "__main__":
    RDK = Robolink()

    # --- simulation settings (use when testing in robodk only) ---
    RDK.setRunMode(RUNMODE_SIMULATE)

    # --- real robot settings (uncomment when running on the physical robot) ---
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # connect to the robot before running: connect -> connect robot, then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    # RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM).RunCode()

    # sim only: puts the rancilio tool in the group head and takes it off the tool stand, as it would be after task l
    RDK.Item("Show_Rancilio_Rancilio_Tool_Rotated", ITEM_TYPE_PROGRAM).RunCode()
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    run_task_s(UR5, tls, RDK)
