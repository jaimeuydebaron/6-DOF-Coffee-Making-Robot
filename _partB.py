# _partB.py
from robodk.robolink import *   # Bridge between Python and RoboDK
from robodk.robomath import *   # Pose-building helpers — transl(), rotx(), roty(), rotz(), pi, eye(), etc.
from time import sleep          # For pauses between actions
import tools                    # Local tools.py — the Tools class (attach/detach, actuation methods)
from __points import POINTS       # Local points.py — the raw survey data dictionary
import math                     # Needed for atan2

# Definitely need an intermediate point for this one before we get there cause we actually collide with the coffe machine before we get to it

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


def compute_task_b_poses():
    """
    Pure calculation only — no robot actions here.
    Task B: use the Mazzer tool to unlock the Mazzer Scale.
    Returns three poses: approach, slid, pressed.
    """
    # --- Fixture pose: Mazzer Scale (planar case) ---
    origin_w = get_point(6)   # Mazzer Scale's origin, world coordinates
    origin_l = get_point(27)  # Mazzer Scale's origin, local coordinates
    ref_w = get_point(7)      # second known point, world coordinates
    ref_l = get_point(28)     # same point, local coordinates

    world_dx = ref_w[0] - origin_w[0]
    world_dy = ref_w[1] - origin_w[1]
    local_dx = ref_l[0] - origin_l[0]
    local_dy = ref_l[1] - origin_l[1]

    cross = local_dx * world_dy - local_dy * world_dx
    dot   = local_dx * world_dx + local_dy * world_dy
    theta = math.atan2(cross, dot)

    mazzer_scale_pose = transl(*origin_w) * rotz(theta)  # the fixture's own verified pose

    # --- Fine-tune the fixture pose itself (EDIT HERE IN THE LAB) ---
    # Corrects for calibration drift between simulation and the real cart,
    # expressed in the Mazzer Scale's own local x/y/z directions.
    FIXTURE_FINE_TUNE_MM = (0, 0, 0)
    mazzer_scale_pose = mazzer_scale_pose * transl(*FIXTURE_FINE_TUNE_MM)

    # --- Placement: lock lever (Key 30 = left lever), axis mapping by inspection ---
    point_local = get_point(30)  # MS_Lock_Lever (left) — swap to 31 for the right lever
    axis_mapping = [[0,1,0], [1,0,0], [0,0,-1]]  # tool axes mapped onto scale axes, by inspection

    # --- Tool offset: Mazzer tool's contact point (Key 51 = cylinder tip) ---
    tool_point_local = get_point(51)
    T_tool_offset = rotz(130 * pi/180) * transl(*tool_point_local)  # mounting-angle offset — unconfirmed, flag for checking

    # --- Approach pose: at the lever, before sliding ---
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_scale_pose * T_fixture_placement
    task_b_approach_pose = T_world_target * T_tool_offset.inv()

    # --- Slide: applied directly to the fixture's own pose, in its own local frame ---
    SLIDE_MM = (-20, 0, 0)  # slide amount, in the Mazzer Scale's own x/y/z directions
    mazzer_scale_pose_slid = mazzer_scale_pose * transl(*SLIDE_MM)
    T_world_target_slid = mazzer_scale_pose_slid * T_fixture_placement
    task_b_slide_pose = T_world_target_slid * T_tool_offset.inv()

    # --- Press: same idea, a further offset along the fixture's own frame ---
    PRESS_MM = (0, 0, -15)  # press downward, matching the 15mm lever-travel dimension from the lab frames diagram
    mazzer_scale_pose_pressed = mazzer_scale_pose_slid * transl(*PRESS_MM)
    T_world_target_pressed = mazzer_scale_pose_pressed * T_fixture_placement
    task_b_press_pose = T_world_target_pressed * T_tool_offset.inv()

    return task_b_approach_pose, task_b_slide_pose, task_b_press_pose


def run_task_b(UR5, tls, RDK):
    """Robot actions only — this is the part likely to change in the lab
    (approach heights, order of moves, added waypoints, etc.)."""
    approach_pose, slide_pose, press_pose = compute_task_b_poses()

    tls.mazzer_tool_attach_r_ati()      # pick up the Mazzer tool from the tool stand
    UR5.MoveJ(approach_pose, True)      # move above/beside the lock lever
    UR5.MoveL(slide_pose, True)         # slide along the lever
    UR5.MoveL(press_pose, True)         # press down to flip the lever
    tls.mazzer_tool_detach_r_ati()      # return the Mazzer tool to the stand
    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)  # return home


if __name__ == "__main__":
    # This block only runs when this file is executed directly (python _partB.py),
    # not when it's imported by another file later.
    RDK = Robolink()
    RDK.setRunMode(RUNMODE_SIMULATE)
    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM).RunCode()

    run_task_b(UR5, tls, RDK)