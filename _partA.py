from robodk.robolink import *   # Bridge between Python and RoboDK — gives you Robolink(), Item types, etc.
from robodk.robomath import *   # Pose-building helpers — transl(), rotx(), roty(), rotz(), pi, etc.
from time import sleep          # For pauses between actions
import tools                    # Your local tools.py — the Tools class (attach/detach, actuation methods)
from __points import POINTS       # Your local points.py — the raw survey data dictionary
from __posecalc import compute_placement_pose # Pose computation file

RDK = Robolink()                        # Connect to the open RoboDK app
RDK.setRunMode(RUNMODE_SIMULATE)        # Simulation only, not the real robot

UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)  # Grab the robot
tls = tools.Tools(RDK)                  # Grab the tool actuation methods

robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM) # Resets the entire simulation
robot_program.RunCode()

# 1. (a) Pick up the Rancilio tool and place it on the Mazzer Scale pan.

tls.rancilio_tool_attach_r_ati() # Attach the rancilio tool in R hand configuration

task_a_pose = compute_placement_pose(
    fixture_name="mazzer_scale",
    point_key=29,
    axis_mapping=[[0,0,1],[0,-1,0],[1,0,0]],
    tool_point_key=54,       # Rancilio tool's ball-bearing indent
    tool_z_offset_deg=-50,   # documented in the primer's tool adapter diagram
)

UR5.MoveJ(task_a_pose, True) # Move to desired pose

# Detach tool in robot
# tls.student_tool_detach() 

# Detach tool in simulation

# Hide the Rancilio tool currently attached to the robot's flange
attached_tool = RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
attached_tool.setVisible(False, False)

# Show the ghost object that already sits there
DetachRancilio = RDK.Item("Show_Mazzer_Scale_Rancilio_Tool", ITEM_TYPE_PROGRAM)
DetachRancilio.RunCode()
DetachRancilio.WaitFinished

UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)