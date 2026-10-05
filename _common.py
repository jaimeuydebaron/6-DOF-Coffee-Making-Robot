# _common.py
# Shared pose and geometry helpers used across _partA.py through _partV.py
from robodk.robolink import ITEM_TYPE_PROGRAM
from robodk.robomath import *
from __points import POINTS
import linecache
import math
import msvcrt
import sys


def get_point(key):
    # Pulls the xyz coordinates for a point out of the points table
    entry = POINTS[key]
    return entry[3], entry[4], entry[5]


def build_pose(rotation_columns, xyz):
    # Builds a pose matrix from three rotation columns and a translation
    col_x, col_y, col_z = rotation_columns
    pose = eye(4)
    for i in range(3):
        pose[i, 0] = col_x[i]
        pose[i, 1] = col_y[i]
        pose[i, 2] = col_z[i]
    pose[0, 3], pose[1, 3], pose[2, 3] = xyz
    return pose


def compute_waypoints(target_pose, offsets):
    # Builds a dict of named waypoints, each derived from target_pose by a
    # translation and rotation offset in the tool's own local frame
    # Offsets are given as name, x y z in mm, then rx ry rz in degrees
    # Each waypoint is computed from target_pose independently, they do not chain off each other
    # The translation is applied first, then the rotation in the order x, y, z
    #
    # Example, backs off 100 mm along the target's z to line up, then does a straight move in:
    #   waypoints = compute_waypoints(target_pose, offsets=[
    #       ("approach", (0, 0, -100), (0, 0, 0)),
    #       ("angled",   (0, 0, -40),  (0, 10, 0)),
    #   ])
    #   UR5.MoveL(waypoints["approach"], True)
    #   UR5.MoveL(target_pose, True)
    waypoints = {}

    for name, translation_mm, angle_deg in offsets:
        angle_offset = rotx(angle_deg[0]*pi/180) \
                     * roty(angle_deg[1]*pi/180) \
                     * rotz(angle_deg[2]*pi/180)
        waypoints[name] = target_pose * transl(*translation_mm) * angle_offset

    return waypoints


def normalize(v):
    # Scales a vector down to unit length
    length = math.sqrt(sum(c**2 for c in v))
    return tuple(c / length for c in v)


def cross_product(a, b):
    # Standard three dimensional cross product
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    )


def rotate_about_tip(pose, tip_offset_mm, angle_adjust):
    # Rotates a pose about the tool's tip rather than about the tcp, so the
    # tip stays put in place while only the tool's facing direction changes
    return pose \
        * transl(*tip_offset_mm) \
        * angle_adjust \
        * transl(*tuple(-c for c in tip_offset_mm))


def solve_frame_rotation(origin_w, origin_l, ref_w, ref_l):
    # Solves for the rotation angle in radians between a fixture's frame and
    # the world frame, given the same origin and reference point measured in both
    world_dx = ref_w[0] - origin_w[0]
    world_dy = ref_w[1] - origin_w[1]
    local_dx = ref_l[0] - origin_l[0]
    local_dy = ref_l[1] - origin_l[1]

    cross = local_dx * world_dy - local_dy * world_dx
    dot   = local_dx * world_dx + local_dy * world_dy
    return math.atan2(cross, dot)


def compute_mazzer_pose():
    # Gets the mazzer's origin and two reference points, one out along the
    # local x axis and one out along the local y axis
    origin_w = get_point(3)
    x_ref_w = get_point(4)
    y_ref_w = get_point(5)

    # Finds the direction from the origin to each reference point in world coordinates
    x_dir = tuple(x_ref_w[i] - origin_w[i] for i in range(3))
    y_dir = tuple(y_ref_w[i] - origin_w[i] for i in range(3))

    # Normalizes both directions into unit vectors, these become the x and y columns of the rotation matrix
    x_axis = normalize(x_dir)
    y_axis = normalize(y_dir)
    x_axis = tuple(-c for c in x_axis)

    # Takes the cross product to get a z axis that keeps the frame right handed
    z_axis = cross_product(x_axis, y_axis)

    mazzer_pose = build_pose([x_axis, y_axis, z_axis], origin_w)
    return mazzer_pose


def solve_joints_near(UR5, pose, config_hint):
    # Solves the joint angles that put the master tool at pose, choosing the solution closest to config_hint
    # Only solutions with the shoulder up and the elbow bent the same way as the hint are kept, so a left hand
    # hint always gives a left hand solution and a right hand hint a right hand one, since wrapped angles
    # (e.g. j3 = 258.5 for -101.5) can otherwise make the other elbow look closer
    solutions = UR5.SolveIK_All(pose, UR5.PoseTool())
    best, best_dist = None, None
    for c in range(solutions.size(1)):
        joints = [solutions[r, c] for r in range(6)]
        shoulder_up = -180 <= joints[1] <= 0
        same_elbow = 0 < joints[2] * config_hint[2] and abs(joints[2]) < 180
        if not (shoulder_up and same_elbow):
            continue
        dist = sum(abs(j - h) for j, h in zip(joints, config_hint))
        if best_dist is None or dist < best_dist:
            best, best_dist = joints, dist
    if best is None:
        raise ValueError("No joint solution for this pose in the hinted configuration, it may be out of reach")
    return best


def run_visual(RDK, program_name):
    # Runs one of the station's visual programs, such as showing or hiding a ghost tool or cup, and waits for it
    # to finish. These only change the simulation, they do not move the robot
    program = RDK.Item(program_name, ITEM_TYPE_PROGRAM)
    program.RunCode()
    program.WaitFinished()


def enable_step_mode(UR5, tls=None):
    # Makes the robot wait for a key press in the terminal before every MoveJ and MoveL, for testing one task on its
    # own. Enter does the move, c carries on to the end without stopping again, escape stops the script before the
    # move. Pass tls as well to step through the tool stand routines, which move through their own robot item
    # Only the __main__ block at the bottom of each task file calls this, so the full run in __main.py never waits
    state = {"paused": True, "count": 0}

    def wait_then(move, kind):
        def stepped(*args, **kwargs):
            state["count"] += 1
            if state["paused"]:
                caller = sys._getframe(1)
                source = linecache.getline(caller.f_code.co_filename, caller.f_lineno).strip()
                print(f"[step {state['count']}] {kind}: {source}")
                print("    enter to move, c to carry on without stopping, escape to stop here")
                while True:
                    key = msvcrt.getch()
                    if key == b"\r":
                        break
                    if key in (b"c", b"C"):
                        state["paused"] = False
                        break
                    if key == b"\x1b":
                        raise SystemExit(f"Stopped before step {state['count']}, the robot has not made this move")
            return move(*args, **kwargs)
        return stepped

    robots = [UR5] if tls is None else [UR5, tls.UR5]
    for robot in robots:
        robot.MoveJ = wait_then(robot.MoveJ, "MoveJ")
        robot.MoveL = wait_then(robot.MoveL, "MoveL")
