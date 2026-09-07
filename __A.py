# Python commands to complete part A of the assignment

from robodk.robolink import * # This imports everything from robolink module
from robodk.robomath import * # gives you pose-building helpers
from time import sleep # This helps us pause execution
import tools # This is the tools file, helps us with table tools

RDK = Robolink() # Creates a link between script and open RoboDK app

tls = tools.Tools(RDK) # Tools class instance

RDK.setRunMode(RUNMODE_SIMULATE) # Says just simulate, if RUNMODE_RUN_ROBOT, will make robot move

UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT) # Link to the robot 


tls.rancilio_tool_attach_r_ati() # Move to and attach rancilio tool

sleep(3) # Buffer
]
# Need to calculate this move to
UR5.MoveJ(RDK.Item("Rancilio_Tool_(Mazzer)", ITEM_TYPE_TARGET), True)  # move to the placement pose