# _partC.py
from robodk.robolink import *   # Bridge between Python and RoboDK
from robodk.robomath import *   # Pose-building helpers — transl(), rotx(), roty(), rotz(), pi, eye(), etc.
from time import sleep          # For pauses between actions
import tools                    # Local tools.py — the Tools class (attach/detach, actuation methods)
from __points import POINTS     # Local points.py — the raw survey data dictionary
import math                     # Needed for sqrt

# Need to change button pressing pose and fine tune the position of the button as it is slightly off, and also move it sort of like backward and then forward. 

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


def compute_mazzer_pose():
    """
    Computes the Mazzer's own frame pose using TWO measured reference
    points (one along local x, one along local y) — since the Mazzer is
    tilted and the simple planar (atan2) method doesn't apply here.

    Key 22 lies along the Mazzer's local x-axis.
    Key 23 lies along the Mazzer's local y-axis.
    The z-axis is computed via cross product, not measured.
    """
    origin_w = get_point(3)   # Mazzer's origin, world coordinates
    x_ref_w = get_point(4)    # Key 4 — world-frame version of Key 22 (local x-axis point)
    y_ref_w = get_point(5)    # Key 5 — world-frame version of Key 23 (local y-axis point)

    # Direction from origin to each reference point, in world coordinates
    x_dir = tuple(x_ref_w[i] - origin_w[i] for i in range(3))
    y_dir = tuple(y_ref_w[i] - origin_w[i] for i in range(3))

    # Normalize to unit vectors — these ARE the x and y columns of the
    # rotation matrix (Section 5: each column is where that axis points)
    x_axis = normalize(x_dir)
    y_axis = normalize(y_dir)
    x_axis = tuple(-c for c in x_axis)  # flip — was pointing the wrong way

    # z-axis is computed, not measured, to keep a valid right-handed frame
    z_axis = cross_product(x_axis, y_axis)

    mazzer_pose = build_pose([x_axis, y_axis, z_axis], origin_w)
    return mazzer_pose


def compute_task_c_pose():
    """
    Pure calculation only — no robot actions here.
    Task C: use the Mazzer tool to turn the Mazzer on, wait 15s, turn off.
    """
    mazzer_pose = compute_mazzer_pose()

    # --- Fine-tune the fixture pose itself (EDIT HERE IN THE LAB) ---
    FIXTURE_FINE_TUNE_MM = (0, 0, 0)
    mazzer_pose = mazzer_pose * transl(*FIXTURE_FINE_TUNE_MM)

    # --- Placement: Mazzer on button (Key 26), axis mapping by inspection ---
    point_local = get_point(26)  # "Mazzer on button centre (raised)"
    axis_mapping = [[0,1,0], [0,0,-1], [-1,0,0]]  # tool axes mapped onto Mazzer axes, by inspection — VERIFY visually
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    # test_frame = RDK.AddFrame("TaskCCheck")
    # test_frame.setParent(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    # test_frame.setPose(T_world_target)   # check before the tool offset is applied

    # --- Tool offset: Mazzer tool's cylinder tip (Key 51) ---
    tool_point_local = get_point(51)
    T_tool_offset = rotz(0 * pi/180) * transl(*tool_point_local)  # mounting-angle offset — unconfirmed, flag for checking
    task_c_pose = T_world_target * T_tool_offset.inv()

    return task_c_pose


def compute_task_c_off_pose():
    """Same as above, but for the OFF button (Key 25)."""
    mazzer_pose = compute_mazzer_pose()

    FIXTURE_FINE_TUNE_MM = (0, 0, 0)
    mazzer_pose = mazzer_pose * transl(*FIXTURE_FINE_TUNE_MM)

    point_local = get_point(25)  # "Mazzer off button centre (raised)"
    axis_mapping = [[0,1,0], [0,0,-1], [-1,0,0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    tool_point_local = get_point(51)
    T_tool_offset = rotz(0 * pi/180) * transl(*tool_point_local)
    return T_world_target * T_tool_offset.inv()


def run_task_c(UR5, tls, RDK):
    """Robot actions only — this is the part likely to change in the lab."""
    on_pose = compute_task_c_pose()
    off_pose = compute_task_c_off_pose()

    tls.mazzer_tool_attach_r_ati()   # pick up the Mazzer tool
    UR5.MoveJ(on_pose, True)         # press the on button
    sleep(15)                        # grind for 15 seconds, per the assignment brief
    UR5.MoveJ(off_pose, True)        # press the off button
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

    run_task_c(UR5, tls, RDK)  # uncomment once MazzerCheck is confirmed correct