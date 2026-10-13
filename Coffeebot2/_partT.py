# _partT.py
# Task t, position the rancilio tool over the rancilio tool cleaner's silicone brush, and actuate it for 5 s
# The basket is lowered from above onto the brush, pressed down to actuate the cleaner, held, then lifted clear
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, solve_frame_rotation, run_visual, enable_space_pause


# Brushes on the rancilio tool cleaner, silicone for task t and bristle for task u
SILICONE_BRUSH = 60
BRISTLE_BRUSH = 61


def compute_tool_cleaner_pose(fixture_fine_tune_mm):
    # The tool cleaner's frame, see part a for how this works
    origin_w = get_point(56)
    origin_l = get_point(58)
    ref_w = get_point(57)
    ref_l = get_point(59)

    theta = solve_frame_rotation(origin_w, origin_l, ref_w, ref_l)

    tool_cleaner_pose = transl(*origin_w) * rotz(theta)

    tool_cleaner_pose = tool_cleaner_pose * transl(*fixture_fine_tune_mm)

    return tool_cleaner_pose


def compute_task_t_pose(brush_key, fixture_fine_tune_mm, approach_angle_adjust_deg, hover_mm=0):
    tool_cleaner_pose = compute_tool_cleaner_pose(fixture_fine_tune_mm)

    # The target is the top of the brush
    point_local = get_point(brush_key)
    axis_mapping = [
        [0, 0, -1],  # Tool x axis lines up with the fixture's negative z axis, the basket opening faces down
        [1, 0, 0],   # Tool y axis lines up with the fixture's x axis
        [0, -1, 0],  # Tool z axis lines up with the fixture's negative y axis, the handle away from the robot
    ]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = tool_cleaner_pose * T_fixture_placement

    # The tool's contact point is the basket rim centre (point 55)
    tool_point_local = get_point(55)
    T_tool_offset = rotz(-50 * pi/180) * transl(*tool_point_local)
    task_t_pose = T_world_target * T_tool_offset.inv()

    # Angle adjust about the tcp
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_t_pose = task_t_pose * angle_adjust

    # Raised in the world frame by hover_mm, negative presses down into the brush
    task_t_pose = transl(0, 0, hover_mm) * task_t_pose

    return task_t_pose


def clean_on_brush(UR5, brush_pose, pressed_pose, hover_pose, clean_time_s):
    # Lowers the basket onto the brush, presses down to actuate the cleaner, holds, then lifts clear
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

    # Shifts where the basket goes over the brushes, along the cleaner's own axes
    fixture_fine_tune_mm = (0, 0, 0)

    # Rotates the tool about the tcp, so the basket moves too
    approach_angle_adjust_deg = (0, 0, 0)

    # Height above the brush where the basket lines up before lowering, and lifts back to
    approach_height_mm = 100

    # How far the basket presses the brush down. The target is the basket rim, so this is 10 mm more than the
    # brush travel, letting the brush reach up into the 29.3 mm deep basket
    press_mm = 26

    # How long the basket is held pressed on the brush, the assignment asks for 5 seconds
    clean_time_s = 5
    # =============================================================================

    # Computes the poses above the silicone brush, on it, and pressing it down
    hover_pose = compute_task_t_pose(SILICONE_BRUSH, fixture_fine_tune_mm, approach_angle_adjust_deg, approach_height_mm)
    brush_pose = compute_task_t_pose(SILICONE_BRUSH, fixture_fine_tune_mm, approach_angle_adjust_deg)
    pressed_pose = compute_task_t_pose(SILICONE_BRUSH, fixture_fine_tune_mm, approach_angle_adjust_deg, -press_mm)

    # Reads the poses in the robot base frame with the master tool
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # Swings over from where task s finishes through the jogged poses to line the basket up above the silicone
    # brush. The basket rolls over on the way, from facing up into the group head to facing down onto the brush
    UR5.MoveL(approach_intermediate_pose_1, True)
    UR5.MoveL(approach_intermediate_pose_3, True)
    UR5.MoveJ(approach_intermediate_pose_2, True)

    # Cleans on the silicone brush
    clean_on_brush(UR5, brush_pose, pressed_pose, hover_pose, clean_time_s)


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)

    # Space in the terminal pauses and plays the run, for testing this task on its own
    enable_space_pause(UR5, tls)

    # RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # Shows the cup on the rancilio scale, where it stays until task r. Simulation only
    run_visual(RDK, "Show_Rancilio_Scale_Cup")

    # Picks up the rancilio tool, in the full run it is already held, carried over from task s
    tls.rancilio_tool_attach_l_ati()

    run_task_t(UR5, tls, RDK)

    # # Returns the rancilio tool and sends the robot home, in the full run task u carries on still holding it
    # tls.rancilio_tool_detach_l_ati()
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
