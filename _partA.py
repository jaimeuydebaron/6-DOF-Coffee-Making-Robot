# _partA.py
from robodk.robolink import *   # Bridge between Python and RoboDK
from robodk.robomath import *   # Pose-building helpers — transl(), rotx(), roty(), rotz(), pi, eye(), etc.
from time import sleep          # For pauses between actions
import tools                    # Local tools.py — the Tools class (attach/detach, actuation methods)
from __points import POINTS     # Local points.py — the raw survey data dictionary
import math                     # Needed for atan2, hypot, etc.


# TA suggested intermediate points and using a cartesian jog to determine where we should place things. Also it collides

def get_point(key):
    entry = POINTS[key]          # look up the row for this key number
    return entry[3], entry[4], entry[5]  # return just the x, y, z coordinates


def build_pose(rotation_columns, xyz):
    col_x, col_y, col_z = rotation_columns  # unpack the three axis-direction columns
    pose = eye(4)                # start with a 4x4 identity matrix
    for i in range(3):
        pose[i, 0] = col_x[i]    # column 1 = where the local x-axis points
        pose[i, 1] = col_y[i]    # column 2 = where the local y-axis points
        pose[i, 2] = col_z[i]    # column 3 = where the local z-axis points
    pose[0, 3], pose[1, 3], pose[2, 3] = xyz  # set the translation part (position)
    return pose


def compute_task_a_pose():
    """Pure calculation only — no robot actions here."""
    # --- Fixture pose: Mazzer Scale (planar case) ---
    origin_w = get_point(6)   # Mazzer Scale's origin, in world/robot coordinates
    origin_l = get_point(27)  # Mazzer Scale's origin, in its own local coordinates (should be ~0,0,0)
    ref_w = get_point(7)      # a second known point on the scale, in world coordinates
    ref_l = get_point(28)     # that same second point, in the scale's own local coordinates

    world_dx = ref_w[0] - origin_w[0]  # x-component of the arrow from origin to ref point, in world frame
    world_dy = ref_w[1] - origin_w[1]  # y-component of that same arrow, in world frame
    local_dx = ref_l[0] - origin_l[0]  # x-component of the same arrow, but in the scale's own frame
    local_dy = ref_l[1] - origin_l[1]  # y-component of the same arrow, in the scale's own frame

    cross = local_dx * world_dy - local_dy * world_dx  # cross-product term, for atan2
    dot   = local_dx * world_dx + local_dy * world_dy  # dot-product term, for atan2
    theta = math.atan2(cross, dot)  # the rotation angle between the two frames, about z

    mazzer_scale_pose = transl(*origin_w) * rotz(theta)  # assemble translation + rotation into one pose

    # --- Placement: ball-bearing apex (Key 29), axis mapping by inspection ---
    point_local = get_point(29)  # the actual placement point, in the scale's own local coordinates
    axis_mapping = [[0,0,1], [0,-1,0], [1,0,0]]  # tool's axes, mapped onto the scale's axes, by inspection
    T_fixture_placement = build_pose(axis_mapping, point_local)  # pose of the placement point+orientation, relative to the scale
    T_world_target = mazzer_scale_pose * T_fixture_placement  # chain onto the scale's world pose

    # --- Tool offset: Rancilio tool's ball-bearing indent (Key 54) ---
    tool_point_local = get_point(54)  # where the tool's own contact point sits, relative to its own origin (TCP)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)  # tool's mounting angle + contact point offset
    task_a_pose = T_world_target * T_tool_offset.inv()  # invert to get where the TCP itself needs to be sent

    # --- Fine-tune (EDIT HERE IN THE LAB) ---
    FINE_TUNE_TRANSLATION_MM = (0, 0, 0)  # small position nudge, in the tool's own current local frame
    FINE_TUNE_ROTATION_DEG   = (0, 0, 0)  # small rotation nudge, about the tool's own current local axes
    fine_tune = transl(*FINE_TUNE_TRANSLATION_MM) \
              * rotx(FINE_TUNE_ROTATION_DEG[0]*pi/180) \
              * roty(FINE_TUNE_ROTATION_DEG[1]*pi/180) \
              * rotz(FINE_TUNE_ROTATION_DEG[2]*pi/180)  # build the fine-tune adjustment as its own small pose

    return task_a_pose * fine_tune  # apply the fine-tune on top of the computed pose, and return the final result


def run_task_a(UR5, tls, RDK):
    """Robot actions only — this is the part likely to change in the lab
    (approach heights, order of moves, added waypoints, etc.)."""
    pose = compute_task_a_pose()  # get the final computed+fine-tuned pose

    tls.rancilio_tool_attach_r_ati()  # pick up the Rancilio tool from the tool stand
    UR5.MoveJ(pose, True)             # move the robot's TCP to the computed pose (blocking — wait until done)
    # tls.student_tool_detach()         # release the tool here, leaving it on the scale
    # UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)

    # # Detach tool in simulation
    # # Hide the Rancilio tool currently attached to the robot's flange
    # attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
    # attached_tool.setVisible(False, False)

    # # Show the ghost object that already sits there
    # DetachRancilio = RDK.Item("Show_Mazzer_Scale_Rancilio_Tool", ITEM_TYPE_PROGRAM)
    # DetachRancilio.RunCode()
    # DetachRancilio.WaitFinished

    # UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)


if __name__ == "__main__":
    # This block only runs when this file is executed directly (python _partA.py),
    # not when it's imported by another file later.
    RDK = Robolink()                        # connect to the open RoboDK app
    RDK.setRunMode(RUNMODE_SIMULATE)        # simulation only, not the real robot
    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)  # grab the robot
    tls = tools.Tools(RDK)                  # grab the tool actuation methods
    RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM).RunCode()  # reset to a known clean starting state

    run_task_a(UR5, tls, RDK)  # actually run task A