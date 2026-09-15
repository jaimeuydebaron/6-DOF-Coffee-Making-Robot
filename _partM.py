# _partM.py
from robodk.robolink import *   # Bridge between Python and RoboDK
from robodk.robomath import *   # Pose-building helpers — transl(), rotx(), roty(), rotz(), pi, eye(), etc.
from time import sleep          # For pauses between actions
import tools                    # Local tools.py — the Tools class (attach/detach, actuation methods)
from __points import POINTS     # Local points.py — the raw survey data dictionary
import math                     # Needed for sqrt


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


def normalize(v):
    length = math.sqrt(sum(c**2 for c in v))
    return tuple(c / length for c in v)


def cross_product(a, b):
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    )


def compute_cup_dispenser_pose():
    """
    Computes the cup dispenser's own frame pose using TWO measured refrence points.

    Key 1 is the cup dispenser frame in the world frame
    Key 2 is a point on the dispenser in the world frame
    Key 16 is the cup dispenser frame in the locak frame
    Key 17 is the point in the dispenser local frame similar to Key 2
    Key 19 is the pull position in the local frame
    Key 28 is the open position in the local frame 
    
    """

    origin_w = get_point(1)
    origin_l = get_point(16)
    ref_w = get_point(2)
    ref_l = get_point(17)

    translation = origin_w

    world_frame_displacement = tuple(ref_w[i] - origin_w[i] for i in range(3))

    frame_tilt = roty(-90 * pi / 180)

    local_point = tuple(ref_l[i] - origin_l[i] for i in range(3)) 

    def apply_rotation(R, point):
        x = R[0,0]*point[0] + R[0,1]*point[1] + R[0,2]*point[2]
        y = R[1,0]*point[0] + R[1,1]*point[1] + R[1,2]*point[2]
        z = R[2,0]*point[0] + R[2,1]*point[1] + R[2,2]*point[2]
        return (x, y, z)

    upright_point = apply_rotation(frame_tilt, local_point)

    cross = upright_point[0] * world_frame_displacement[1] - upright_point[1] * world_frame_displacement[0]
    dot = upright_point[0] * world_frame_displacement[0] + upright_point[1] * world_frame_displacement[1]

    psi = math.atan2(cross, dot)

    UR_R_CSprime = rotz(psi)
    UR_R_CS = UR_R_CSprime * frame_tilt

    cup_dispenser_pose = transl(*translation) * UR_R_CS

    return cup_dispenser_pose

def compute_task_m_shut_pose():

    cup_dispenser_pose = compute_cup_dispenser_pose()

    # --- Fine-tune the fixture pose itself (EDIT HERE IN THE LAB) ---
    FIXTURE_FINE_TUNE_MM = (0, 0, 0)
    mazzer_pose = cup_dispenser_pose * transl(*FIXTURE_FINE_TUNE_MM)

    # --- Placement: Mazzer on button (Key 26), axis mapping by inspection ---
    point_local = get_point(19)  # "Mazzer on button centre (raised)"
    axis_mapping = [[0,1,0], [0,0,-1], [-1,0,0]]  # tool axes mapped onto Mazzer axes, by inspection — VERIFY visually
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    # test_frame = RDK.AddFrame("TaskCCheck")
    # test_frame.setParent(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    # test_frame.setPose(T_world_target)   # check before the tool offset is applied

    # --- Tool offset: Mazzer tool's cylinder tip (Key 51) ---
    tool_point_local = get_point(51)
    T_tool_offset = rotz(0 * pi/180) * transl(*tool_point_local)  # mounting-angle offset — unconfirmed, flag for checking
    task_m_shut_pose_part1 = T_world_target * T_tool_offset.inv()

    return task_m_shut_pose_part1


def compute_task_m_open_pose():
    """Same as above, but for the OFF button (Key 25)."""
    cup_dispenser_pose = compute_cup_dispenser_pose()


    FIXTURE_FINE_TUNE_MM = (0, 0, 0)
    cup_dispenser_pose = cup_dispenser_pose * transl(*FIXTURE_FINE_TUNE_MM)

    point_local = get_point(18) 
    axis_mapping = [[0,1,0], [0,0,-1], [-1,0,0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = cup_dispenser_pose * T_fixture_placement

    tool_point_local = get_point(51)
    T_tool_offset = rotz(0 * pi/180) * transl(*tool_point_local)
    return T_world_target * T_tool_offset.inv()


def run_task_m(UR5, tls, RDK):
    """Robot actions only — this is the part likely to change in the lab."""
    shut_pose = compute_task_m_shut_pose()
    open_pose = compute_task_m_open_pose()

    tls.mazzer_tool_attach_r_ati()   # pick up the Mazzer tool
    UR5.MoveJ(shut_pose, True)         # press the on button
    sleep(3)                        # grind for 15 seconds, per the assignment brief
    UR5.MoveJ(open_pose, True)        # press the off button
    tls.mazzer_tool_detach_r_ati()   # return the tool
    # UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)  # go home


if __name__ == "__main__":
    RDK = Robolink()
    RDK.setRunMode(RUNMODE_SIMULATE)
    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM).RunCode()

    # --- Visual check before running the real task ---
    # mazzer_pose = compute_mazzer_pose()
    # test_frame = RDK.AddFrame("MazzerCheck")
    # test_frame.setParent(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    # test_frame.setPose(mazzer_pose)

    run_task_m(UR5, tls, RDK)  # uncomment once MazzerCheck is confirmed correct