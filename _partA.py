# _partA.py
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


def compute_task_a_pose(fixture_fine_tune_mm, approach_angle_adjust_deg):
    # gets the mazzer scale origin and reference point in world and local coordinates
    origin_w = get_point(6)
    origin_l = get_point(27)
    ref_w = get_point(7)
    ref_l = get_point(28)

    # finds the displacement between origin and reference point in both frames
    world_dx = ref_w[0] - origin_w[0]
    world_dy = ref_w[1] - origin_w[1]
    local_dx = ref_l[0] - origin_l[0]
    local_dy = ref_l[1] - origin_l[1]

    # solves for the rotation angle between the two frames
    cross = local_dx * world_dy - local_dy * world_dx
    dot   = local_dx * world_dx + local_dy * world_dy
    theta = math.atan2(cross, dot)

    # builds the mazzer scale's pose in the robot's world frame
    mazzer_scale_pose = transl(*origin_w) * rotz(theta)

    # applies the fixture calibration correction
    mazzer_scale_pose = mazzer_scale_pose * transl(*fixture_fine_tune_mm)

    # sets the placement point as the ball bearing apex and the tool's orientation there, by inspection
    point_local = get_point(29)
    axis_mapping = [[0,0,1], [0,-1,0], [1,0,0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_scale_pose * T_fixture_placement

    # sets the rancilio tool's contact point and mounting angle offset
    tool_point_local = get_point(54)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    task_a_pose = T_world_target * T_tool_offset.inv()

    # applies the approach angle correction
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_a_pose = task_a_pose * angle_adjust

    return task_a_pose

approach_general_point1 = [94.213014, -105.631792, 119.134449, -103.512723, -89.997053, -230.793487]
approach_general_point2 = [148.740997, -103.856520, 121.433962, -17.577449, 118.713599, 139.999989]
approach_general_point3 = [148.741161, -82.451284, 138.999911, -56.548628, 118.714037, 140.000000]


def run_task_a(UR5, tls, RDK):
    # edit these in the lab

    # shifts the mazzer scale's whole pose in its own local x, y, z directions to correct calibration drift
    fixture_fine_tune_mm = (0, 0, 0)

    # rotates the tool's approach about its own x, y, z axes once already oriented at the placement point
    approach_angle_adjust_deg = (0, 0, 0)

    task_a_pose = compute_task_a_pose(fixture_fine_tune_mm, approach_angle_adjust_deg)

    waypoints = compute_waypoints(task_a_pose, offsets=[
        ("approach", (0, 0, -100), (0, 0, 0)),
    ])

    # picks up the rancilio tool from the tool stand
    tls.rancilio_tool_attach_l_ati()
    UR5.MoveJ(approach_general_point1, True)
    UR5.MoveJ(approach_general_point2, True)
    UR5.MoveL(approach_general_point3, True)



    # # moves in and places the tool on the scale
    # UR5.MoveL(waypoints["approach"], True)
    UR5.MoveL(task_a_pose, True)

    # # releases the tool, showing the ghost object left in its place
    # tls.student_tool_detach()
    attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
    attached_tool.setVisible(False, False)

    detach_rancilio = RDK.Item("Show_Mazzer_Scale_Rancilio_Tool", ITEM_TYPE_PROGRAM)
    detach_rancilio.RunCode()
    detach_rancilio.WaitFinished()

    # # retreats and sends the robot home
    # UR5.MoveL(waypoints["approach"], True)
    UR5.MoveL(approach_general_point3, True)
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

    run_task_a(UR5, tls, RDK)