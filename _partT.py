# _partT.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from __points import POINTS
from _partS import run_task_s
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


# brushes on the rancilio tool cleaner, silicone for task t and bristle for task u
SILICONE_BRUSH = 60
BRISTLE_BRUSH = 61


def compute_cleaner_pose(brush_key, fixture_fine_tune_mm, approach_angle_adjust_deg):
    # gets the rancilio tool cleaner origin and reference point in world and local coordinates
    origin_w = get_point(56)
    origin_l = get_point(58)
    ref_w = get_point(57)
    ref_l = get_point(59)

    # finds the displacement between origin and reference point in both frames
    world_dx = ref_w[0] - origin_w[0]
    world_dy = ref_w[1] - origin_w[1]
    local_dx = ref_l[0] - origin_l[0]
    local_dy = ref_l[1] - origin_l[1]

    # solves for the rotation angle between the two frames
    cross = local_dx * world_dy - local_dy * world_dx
    dot   = local_dx * world_dx + local_dy * world_dy
    theta = math.atan2(cross, dot)

    # builds the tool cleaner's pose in the robot's world frame
    cleaner_pose = transl(*origin_w) * rotz(theta)

    # applies the fixture calibration correction
    cleaner_pose = cleaner_pose * transl(*fixture_fine_tune_mm)

    # sets the placement point as the brush top and the tool's orientation there, by inspection
    # tool x points down so the basket opening faces the brush, tool z runs along the handle away from the robot (fixture -y)
    point_local = get_point(brush_key)
    axis_mapping = [[0,0,-1], [1,0,0], [0,-1,0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = cleaner_pose * T_fixture_placement

    # sets the rancilio tool's basket rim centre and mounting angle offset
    tool_point_local = get_point(55)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    cleaner_tcp_pose = T_world_target * T_tool_offset.inv()

    # applies the approach angle correction
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    cleaner_tcp_pose = cleaner_tcp_pose * angle_adjust

    return cleaner_tcp_pose


def closest_ik_solution(UR5, pose, seed):
    # solves every joint configuration for the pose and keeps the one closest to the seed,
    # so the joint move does not flip the shoulder or elbow
    # only solutions with the shoulder up and the elbow bent the same way as the seed are kept,
    # since wrapped angles (e.g. j3 = 258.5 for -101.5) can make the other elbow look closer
    # the tool is passed in so the solutions are for the tcp, not the bare flange
    solutions = UR5.SolveIK_All(pose, UR5.PoseTool())
    best, best_dist = None, None
    for c in range(solutions.size(1)):
        joints = [solutions[r, c] for r in range(6)]
        shoulder_up = -180 <= joints[1] <= 0
        same_elbow = 0 < joints[2] * seed[2] and abs(joints[2]) < 180
        if not (shoulder_up and same_elbow):
            continue
        dist = sum(abs(j - s) for j, s in zip(joints, seed))
        if best_dist is None or dist < best_dist:
            best, best_dist = joints, dist
    return best


# edit these in the lab, shared by tasks t and u

# shifts the tool cleaner's whole pose in its own local x, y, z directions to correct calibration drift
fixture_fine_tune_mm = (0, 0, 0)

# rotates the tool's approach about its own x, y, z axes once already oriented at the brush
approach_angle_adjust_deg = (0, 0, 0)

# how high above the brushes the basket travels, how far it presses each brush down, and how long it cleans for
# the target point is the basket rim (top), so the press is 10 mm more than the brush travel,
# letting the brush reach up into the bottom of the basket (basket depth is 29.3)
approach_height_mm = 100
press_mm = 20
clean_time_s = 5


def clean_on_brush(UR5, brush_key):
    # lowers the basket from above the brush onto it, presses down to actuate the cleaner,
    # holds for the cleaning time, then lifts back up clear of the brush
    # vertical moves are done in the world frame, since the tcp is twisted from the tool
    brush_pose = compute_cleaner_pose(brush_key, fixture_fine_tune_mm, approach_angle_adjust_deg)
    above_pose = transl(0, 0, approach_height_mm) * brush_pose
    pressed_pose = transl(0, 0, -press_mm) * brush_pose

    UR5.MoveL(brush_pose, True)
    UR5.MoveL(pressed_pose, True)
    sleep(clean_time_s)
    UR5.MoveL(above_pose, True)


def above_brush(brush_key):
    # pose with the basket lined up above a brush, at the travel height
    brush_pose = compute_cleaner_pose(brush_key, fixture_fine_tune_mm, approach_angle_adjust_deg)
    return transl(0, 0, approach_height_mm) * brush_pose


def run_task_t(UR5, tls, RDK):
    # starts where task s finishes, pulled back from the rancilio, and swings over to line the basket up
    # above the silicone brush, picking the left hand configuration closest to where the arm already is
    # the basket rolls over on the way, from facing up into the group head to facing down onto the brush
    UR5.MoveJ(closest_ik_solution(UR5, above_brush(SILICONE_BRUSH), UR5.Joints().list()), True)

    # cleans on the silicone brush
    clean_on_brush(UR5, SILICONE_BRUSH)

    # finishes above the silicone brush, ready to slide across to the bristle brush in task u


if __name__ == "__main__":
    RDK = Robolink()

    # --- simulation settings (use when testing in robodk only) ---
    RDK.setRunMode(RUNMODE_SIMULATE)

    # --- real robot settings (uncomment when running on the physical robot) ---
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # connect to the robot before running: connect -> connect robot, then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # sim only: puts the rancilio tool in the group head and takes it off the tool stand, as it would be after task l
    RDK.Item("Show_Rancilio_Rancilio_Tool_Rotated", ITEM_TYPE_PROGRAM).RunCode()
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    # task t carries on from where task s finishes, so both run back to back
    run_task_s(UR5, tls, RDK)
    run_task_t(UR5, tls, RDK)

    # sends the robot home when task t is run on its own
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
