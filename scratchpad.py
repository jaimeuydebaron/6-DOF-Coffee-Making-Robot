from robodk.robolink import * # This imports everything from robolink module
from robodk.robomath import * # gives you pose-building helpers
from time import sleep # This helps us pause execution

RDK = Robolink() # Creates a link between script and open RoboDK app

RDK.setRunMode(RUNMODE_SIMULATE) # Says just simulate, if RUNMODE_RUN_ROBOT, will make robot move

UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT) # Link to the robot 

homeR = RDK.Item("Home_R", ITEM_TYPE_TARGET) # Link homeR to Home_R item in the tree, under Table_Top
homeL = RDK.Item("Home_L", ITEM_TYPE_TARGET) # Link homeL to Home_L

# for count in range(1,6):
#     print(count)
#     UR5.MoveJ(homeR, True) # This is a joint move, moves the robot by commanding each joint's angle to change from current value to a target value, all joints rotate simultaneously
#     sleep(5)
#     UR5.MoveJ(homeL, True)
#     sleep(5)

UR5.MoveL(homeR, True) # This is a linear move, moves the robot in a straight line in space


pose = transl(-100, -100, 100) * rotz(60 * pi/180)
frame = RDK.AddFrame("TestFrame")
frame.setPose(pose) # This sets a marker in code to see where it is

UR5.MoveL(pose, True) # Then pass this thru a MoveJ