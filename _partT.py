# _partT.py
# Task t, position the rancilio tool over the rancilio tool cleaner's silicone brush, and actuate it for 5 s
# The basket is lowered from above onto the brush, pressed down to actuate the cleaner, held, then lifted clear
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, solve_frame_rotation, run_visual


# Brushes on the rancilio tool cleaner, silicone for task t and bristle for task u
SILICONE_BRUSH = 60
BRISTLE_BRUSH = 61


def compute_tool_cleaner_pose(fixture_fine_tune_mm):
    # Gets the rancilio tool cleaner's origin and a reference point, in both world and local coordinates
    origin_w = get_point(56)
    origin_l = get_point(58)
    ref_w = get_point(57)
    ref_l = get_point(59)

    # Solves for how far the tool cleaner is rotated relative to the world frame
    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    # Builds the tool cleaner's pose in the robot's world frame
    tool_cleaner_pose = transl(*origin_w) * rotz(theta)

    # Applies the fixture calibration correction
    tool_cleaner_pose = tool_cleaner_pose * transl(*fixture_fine_tune_mm)

    return tool_cleaner_pose


def compute_task_t_pose(brush_key, fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm=0):
    tool_cleaner_pose = compute_tool_cleaner_pose(fixture_fine_tune_mm)

    # Sets the placement point as the top of the brush, with the tool orientation found by inspection
    point_local = get_point(brush_key)
    axis_mapping = [
        [0, 0, -1],  # Tool x axis lines up with the fixture's negative z axis, the basket opening faces down
        [1, 0, 0],   # Tool y axis lines up with the fixture's x axis
        [0, -1, 0],  # Tool z axis lines up with the fixture's negative y axis, the handle away from the robot
    ]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = tool_cleaner_pose * T_fixture_placement

    # Sets the rancilio tool's contact point, the basket rim centre (point 55), and its mounting angle offset
    tool_point_local = get_point(55)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    task_t_pose = T_world_target * T_tool_offset.inv()

    # Applies the approach angle correction
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_t_pose = task_t_pose * angle_adjust

    # Lifts the pose straight up in the world frame by hover_mm, negative presses down into the brush. The world
    # frame is used since the tcp is twisted from the tool
    task_t_pose = transl(0, 0, hover_mm) * task_t_pose

    return task_t_pose


def clean_on_brush(UR5, brush_pose, pressed_pose, hover_pose, clean_time_s):
    # Lowers the basket onto the brush, presses down to actuate the cleaner, holds for the cleaning time, then lifts
    # back up clear of the brush
    UR5.MoveL(brush_pose, True)
    UR5.MoveL(pressed_pose, True)
    sleep(clean_time_s)
    UR5.MoveL(hover_pose, True)

# Jogged joint waypoints that swing the rancilio tool over from the rancilio to the tool cleaner, left hand. The
# second is above the silicone brush with the basket facing down, where the straight move down onto the brush starts
approach_intermediate_pose_1 = [49.292637, -108.370993, 149.547431, -44.690739, 124.328180, 139.219903]
approach_intermediate_pose_2 = [41.903731, -92.293742, 107.559080, -15.265339, 41.903731, -40.000000]
approach_intermediate_pose_3 = [60.375821, -100.119664, 118.434991, -22.157070, 85.418068, 139.053302]

def run_task_t(UR5, tls, RDK):
    """ Does not return the robot home, starts where task s finishes, pulled back from the rancilio, and ends
    holding the rancilio tool above the silicone brush, ready to slide across to the bristle brush in task u """

    # ==================== FINE TUNING (edit these in the lab) ====================
    # Part u cleans on the bristle brush the same way, so these MUST equal the values in run_task_u in _partU.py.

    # fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the basket goes over the brushes, by shifting the assumed position of the tool cleaner along the
    # cleaner's own axes. The tool keeps its angle and just moves as a whole.
    #   x: shifts along the cleaner's local x
    #   y: shifts along the cleaner's local y
    #   z: raises (+) or lowers (-) the basket over the brushes
    fixture_fine_tune_mm = (0, 0, -1)

    # approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z) once it is lined up with the brush.
    # It pivots about the tool's origin, NOT the basket rim centre, so changing it also moves where the basket goes.
    approach_angle_adjust_deg = (0, 0, 0)

    # approach_height_mm
    # How high above the brush the basket lines up before lowering onto it, and lifts back to after.
    approach_height_mm = 100

    # press_mm
    # How far the basket presses the brush down once on it. The target point is the basket rim, so the press is
    # 10 mm more than the brush travel, letting the brush reach up into the bottom of the basket (basket depth is
    # 29.3 mm).
    press_mm = 20

    # clean_time_s
    # How long the basket is held pressed on the brush, the assignment asks for 5 seconds.
    clean_time_s = 5

    # The joint poses above (approach_intermediate_pose_1/2) are jogged joint angles, not affected by the settings
    # here. approach_intermediate_pose_2 is where the straight move down onto the brush starts from, so if you move
    # the fixture or change the angle or the approach height a lot, re-jog it.
    # =============================================================================

    # Computes the poses above the silicone brush, on it, and pressing it down
    hover_pose = compute_task_t_pose(SILICONE_BRUSH, fixture_fine_tune_mm, approach_angle_adjust_deg, approach_height_mm)
    brush_pose = compute_task_t_pose(SILICONE_BRUSH, fixture_fine_tune_mm, approach_angle_adjust_deg)
    pressed_pose = compute_task_t_pose(SILICONE_BRUSH, fixture_fine_tune_mm, approach_angle_adjust_deg, -press_mm)

    # Swings over from where task s finishes through the jogged poses to line the basket up above the silicone
    # brush. The basket rolls over on the way, from facing up into the group head to facing down onto the brush
    # UR5.MoveJ(approach_intermediate_pose_1, True)
    # UR5.MoveJ(approach_intermediate_pose_3, True)
    UR5.MoveJ(approach_intermediate_pose_2, True)

    # # Cleans on the silicone brush
    clean_on_brush(UR5, brush_pose, pressed_pose, hover_pose, clean_time_s)

    # Sends the robot home
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)
    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # Puts the rancilio tool in the group head and takes it off the tool stand, as it would be after task l.
    # Simulation only
    run_visual(RDK, "Show_Rancilio_Rancilio_Tool_Rotated")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)

    run_task_t(UR5, tls, RDK)
