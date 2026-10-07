# _common.py
# Shared pose and geometry helpers used across _partA.py through _partV.py
from robodk.robolink import Robolink, StoppedError, ITEM_TYPE_PROGRAM, ITEM_TYPE_TOOL, RUNMODE_SIMULATE, \
    COLLISION_ON, COLLISION_OFF
from robodk.robomath import *
from __points import POINTS
import _thread
import atexit
import linecache
import math
import os
import msvcrt
import sys
import threading
import time


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
    # Builds named waypoints off target_pose. Each offset is (name, (x, y, z) mm, (rx, ry, rz) deg) in the tool's
    # own frame, translation first, then rotation about x, y, z. Each comes from target_pose on its own, they
    # do not chain
    #   waypoints = compute_waypoints(target_pose, offsets=[("approach", (0, 0, -100), (0, 0, 0))])
    #   UR5.MoveL(waypoints["approach"], True)
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
    # Rotates a pose about the tool's tip instead of the tcp, so the tip stays put and only its facing changes
    return pose \
        * transl(*tip_offset_mm) \
        * angle_adjust \
        * transl(*tuple(-c for c in tip_offset_mm))


def solve_frame_rotation(origin_w, origin_l, ref_w, ref_l):
    # Angle in radians of a fixture's frame about the world z, from an origin and a reference point that are
    # both known in world and in local coordinates
    world_dx = ref_w[0] - origin_w[0]
    world_dy = ref_w[1] - origin_w[1]
    local_dx = ref_l[0] - origin_l[0]
    local_dy = ref_l[1] - origin_l[1]

    cross = local_dx * world_dy - local_dy * world_dx
    dot   = local_dx * world_dx + local_dy * world_dy
    return math.atan2(cross, dot)


def compute_mazzer_pose():
    # The mazzer's frame, from its origin and reference points out along its local x and y (points 3, 4, 5)
    origin_w = get_point(3)
    x_ref_w = get_point(4)
    y_ref_w = get_point(5)

    # Directions from the origin to each reference point, in world coordinates
    x_dir = tuple(x_ref_w[i] - origin_w[i] for i in range(3))
    y_dir = tuple(y_ref_w[i] - origin_w[i] for i in range(3))

    # Unit vectors for the x and y columns of the rotation, x is flipped since point 4 lies along the local -x
    x_axis = normalize(x_dir)
    y_axis = normalize(y_dir)
    x_axis = tuple(-c for c in x_axis)

    # z from the cross product, which keeps the frame right handed
    z_axis = cross_product(x_axis, y_axis)

    mazzer_pose = build_pose([x_axis, y_axis, z_axis], origin_w)
    return mazzer_pose


def solve_joints_near(UR5, pose, config_hint):
    # Joint solution for pose closest to config_hint, keeping only shoulder up solutions with the elbow bent the
    # same way as the hint. Wrapped angles (e.g. j3 = 258.5 for -101.5) can otherwise make the other elbow look closer
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
    # Runs a station visual program (shows or hides a ghost tool or cup) and waits for it. Simulation only
    program = RDK.Item(program_name, ITEM_TYPE_PROGRAM)
    program.RunCode()
    program.WaitFinished()


def enable_space_pause(UR5, tls=None):
    # Space in the terminal pauses the run, space again plays it. In simulation the pause freezes the robot mid
    # move, by dropping the simulation speed to its slowest. On the real robot it holds before the next move
    # instead. Pass tls to also hold the tool stand routines. Click the terminal first, the keys are read from it
    simulating = UR5.link.RunMode() == RUNMODE_SIMULATE
    playing = threading.Event()
    playing.set()

    # The key listener needs its own connection, the main one is busy for as long as a move is running
    RDK = Robolink()
    normal_speed = RDK.SimulationSpeed()
    if normal_speed < 0.01:
        # Left over from a run that was killed while paused
        normal_speed = 1
        RDK.setSimulationSpeed(normal_speed)
    atexit.register(RDK.setSimulationSpeed, normal_speed)

    def listen():
        while True:
            if not msvcrt.kbhit():
                time.sleep(0.05)
                continue
            key = msvcrt.getch()
            if key == b"\x03":
                _thread.interrupt_main()
            if key != b" ":
                continue
            if playing.is_set():
                playing.clear()
                if simulating:
                    RDK.setSimulationSpeed(0.001)
                print("[paused] space to play")
            else:
                if simulating:
                    RDK.setSimulationSpeed(normal_speed)
                playing.set()
                print("[playing] space to pause")

    def hold_then(move):
        def held(*args, **kwargs):
            playing.wait()
            return move(*args, **kwargs)
        return held

    robots = [UR5] if tls is None else [UR5, tls.UR5]
    for robot in robots:
        robot.MoveJ = hold_then(robot.MoveJ)
        robot.MoveL = hold_then(robot.MoveL)

    threading.Thread(target=listen, daemon=True).start()
    print("[pause] Space pauses and plays the run, click this terminal first")


def enable_collision_report(UR5, tls=None):
    # Simulation only. Turns collision checking on and prints every move where the robot, or the tool it holds,
    # touches something, with what touched. Returns the list of contacts, which fills up as the run goes.
    # RoboDK stops a move at its first contact, so that move is then finished with checking off, and only the
    # first contact of each move is caught. Contacts that are meant to happen are listed too, a tool sitting in
    # its fixture or pressing a button, so read the list for the ones that are not
    # The first time in a station RoboDK takes a while to prepare, hence its own connection with a long timeout
    RDK = Robolink()
    RDK.COM.settimeout(300)

    # The tools are mounted on the flange and on each other, so they always touch. Those pairs are left out of
    # the station's collision map, or every move would stop before it started
    robot = RDK.Item(UR5.Name())
    tools_on_arm = RDK.ItemList(ITEM_TYPE_TOOL)
    for i, tool in enumerate(tools_on_arm):
        for link in range(7):
            RDK.setCollisionActivePair(COLLISION_OFF, tool, robot, 0, link)
        for other in tools_on_arm[i + 1:]:
            RDK.setCollisionActivePair(COLLISION_OFF, tool, other)

    RDK.setCollisionActive(COLLISION_ON)
    atexit.register(RDK.setCollisionActive, COLLISION_OFF)

    contacts = []
    state = {"touching": None}

    def check_then(move):
        def checked(*args, **kwargs):
            try:
                result = move(*args, **kwargs)
                state["touching"] = None
                return result
            except StoppedError as exc:
                if "ollision" not in str(exc):
                    raise

            touching = sorted({" and ".join(sorted((a.Name(), b.Name()))) for a, b, _, _ in RDK.CollisionPairs()})

            # A contact carried over from the move before is only reported once
            if touching != state["touching"]:
                caller = sys._getframe(1)
                while os.path.abspath(caller.f_code.co_filename) == os.path.abspath(__file__):
                    caller = caller.f_back
                where = f"{os.path.basename(caller.f_code.co_filename)}:{caller.f_lineno}"
                source = linecache.getline(caller.f_code.co_filename, caller.f_lineno).strip()
                contacts.append((where, source, touching))
                print(f"[collision] {where}  {source}")
                for pair in touching:
                    print(f"    {pair}")
            state["touching"] = touching

            # Finishes the move with checking off, then turns it back on for the next one
            RDK.setCollisionActive(COLLISION_OFF)
            try:
                return move(*args, **kwargs)
            finally:
                RDK.setCollisionActive(COLLISION_ON)
        return checked

    robots = [UR5] if tls is None else [UR5, tls.UR5]
    for item in robots:
        item.MoveJ = check_then(item.MoveJ)
        item.MoveL = check_then(item.MoveL)

    print("[collision] Collision checking is on, every contact is printed as it happens")
    return contacts
