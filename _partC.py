from robodk.robolink import *   # Bridge between Python and RoboDK — gives you Robolink(), Item types, etc.
from robodk.robomath import *   # Pose-building helpers — transl(), rotx(), roty(), rotz(), pi, etc.
from time import sleep          # For pauses between actions
import tools                    # Your local tools.py — the Tools class (attach/detach, actuation methods)
from __points import POINTS       # Your local points.py — the raw survey data dictionary
from __posecalc import compute_placement_pose, compute_fixture_pose # Pose computation file

RDK = Robolink()                        # Connect to the open RoboDK app
RDK.setRunMode(RUNMODE_SIMULATE)        # Simulation only, not the real robot

UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)  # Grab the robot
tls = tools.Tools(RDK)                  # Grab the tool actuation methods

robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM) # Resets the entire simulation
robot_program.RunCode()

# 3. **(c) Use the Mazzer tool to turn the Mazzer on, wait 15s, and turn the Mazzer off.**

# tls.mazzer_tool_attach_r_ati()

# Phase 1: approach — tool positioned above/beside the lever, before sliding in
# task_c_approach_pose = compute_placement_pose(
#     fixture_name="mazzer",
#     point_key=26,              # MS_Lock_Lever (left) — or 31 for right
#     # axis_mapping=[[0,0,1],
#     #               [0,-1,0],
#     #               [1,0,0]],  # needs re-deriving by inspection for this motion
#     axis_mapping=[[0,1,0],
#                   [0,0,1],
#                   [1,0,0]],  # needs re-deriving by inspection for this motion
#     tool_point_key=51,         # Mazzer tool's point
#     tool_z_offset_deg=0,       # unconfirmed for Mazzer tool — flag until verified
# )

# # test_frame = RDK.AddFrame("TestFrame")
# # test_frame.setParent(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
# # test_frame.setPose(task_b_approach_pose)

# UR5.MoveJ(task_c_approach_pose, True) # Move to desired pose, above the lock


# # Then slide it
# task_b_pose_slide = task_b_approach_pose * transl(15, 15, 0) 

# UR5.MoveL(task_b_pose_slide, True)

# # Then press it
# task_b_pose_press = task_b_pose_slide * transl(0, 0, 15) 

# UR5.MoveL(task_b_pose_press, True)

# tls.mazzer_tool_detach_r_ati() # Detach mazzer tool

# UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)

mazzer_pose = compute_fixture_pose("mazzer", world_second_key=5, local_second_key=22)

test_frame = RDK.AddFrame("MazzerCheck")
test_frame.setParent(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
test_frame.setPose(mazzer_pose)
