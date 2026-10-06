# __main.py
# Runs every task, a to v, back to back in one continuous simulation or robot run
# The robot starts and ends at home_l. Each task carries on from where the one before it finishes, so the tasks
# must run in this order, see the comment beside each one for what it is holding when it hands over
#
# Before running, check the scale settings in the tasks that read a scale, they are not set from here:
#   _partD.py run_task_d: use_scale, True reads the real mazzer scale, False pulls the lever once
#   _partP.py run_task_p: use_scale, True reads the real rancilio scale, False runs the water for a fixed time
# Set both False for a simulation run, both True in the lab
from robodk.robolink import *
from robodk.robomath import *
import tools
from _common import run_visual, enable_space_pause, enable_collision_report

from _partA import run_task_a
from _partB import run_task_b
from _partC import run_task_c
from _partD import run_task_d
from _partE import run_task_e
from _partF import run_task_f
from _partGH import run_task_g
from _partI import run_task_i
from _partJ import run_task_j
from _partK import run_task_k
from _partL import run_task_l
from _partO import run_task_o
from _partP import run_task_p
from _partQ import run_task_q
from _partR import run_task_r
from _partS import run_task_s
from _partT import run_task_t
from _partU import run_task_u
from _partV import run_task_v


# Transition waypoints, for clearing a specific task to task handoff that isn't part of either task's own motion.
# Name them by the transition they serve, then slot them into the sequence below as a ("waypoint", ...) entry
# example: e_to_f_transition_joints = [j1, j2, j3, j4, j5, j6]


# True turns collision checking on for a simulation run, and prints every move where the robot or the tool it
# holds touches something. The run is slower with it on, and it is ignored on the real robot
CHECK_COLLISIONS = False


def main():
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)

    # Resets the simulation once for the whole run, the left hand variant since the whole run is left handed.
    # Simulation only, comment this out on the real robot
    run_visual(RDK, "Reset_Simulation_L")

    # Makes sure every pose is read in the robot base frame with the master tool, as the assignment requires
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # Each entry is either ("task", name, description, run_task_x) or ("waypoint", name, description, joints_or_pose)
    # The description is printed in the terminal as each step starts and finishes
    sequence = [
        # Starts at home_l, places the rancilio tool on the mazzer scale, ends empty handed
        ("task", "a", "place the rancilio tool on the mazzer scale", run_task_a),
        # Picks up the mazzer tool and unlocks the mazzer scale, ends HOLDING the mazzer tool
        ("task", "b", "unlock the mazzer scale", run_task_b),
        # Turns the mazzer on and off, ends HOLDING the mazzer tool
        ("task", "c", "turn the mazzer on and off", run_task_c),
        # Doses the grounds, ends HOLDING the mazzer tool
        ("task", "d", "dose the grounds", run_task_d),
        # Task e, with task m run inside it while the mazzer tool is still held. Locks the mazzer scale and
        # dispenses the cup (m), ends with the mazzer tool back on the stand. Do NOT add m here as well, the cup
        # would be dispensed twice
        ("task", "e+m", "lock the mazzer scale, dispense the cup", run_task_e),
        # Picks the rancilio tool up off the mazzer scale, ends HOLDING the rancilio tool
        ("task", "f", "pick the rancilio tool up off the mazzer scale", run_task_f),
        # Tasks g and h, places the rancilio tool in the wdt and shuts it, ends empty handed at home_l
        ("task", "g+h", "place the rancilio tool in the wdt and shut it", run_task_g),
        # Turns the wdt rotor, starts at home_l and ends with the mazzer tool back on the stand
        ("task", "i", "turn the wdt rotor", run_task_i),
        # Takes the rancilio tool out of the wdt, ends HOLDING the rancilio tool
        ("task", "j", "take the rancilio tool out of the wdt", run_task_j),
        # Tamps in the puq, ends HOLDING the rancilio tool, backed out of the puq
        ("task", "k", "tamp in the puq", run_task_k),
        # Task l, with task n run inside it. Sets the rancilio tool down on the mazzer scale, places the cup on the
        # rancilio scale (n), then picks the rancilio tool back up and locks it into the group head. Ends empty
        # handed above the rancilio. Do NOT add n here as well, the cup would be placed twice
        ("task", "l+n", "place the cup on the rancilio scale, lock the rancilio tool into the group head", run_task_l),
        # Unlocks the rancilio scale, ends HOLDING the mazzer tool
        ("task", "o", "unlock the rancilio scale", run_task_o),
        # Runs the hot water into the cup, ends HOLDING the mazzer tool
        ("task", "p", "run the hot water into the cup", run_task_p),
        # Locks the rancilio scale, ends with the mazzer tool back on the stand
        ("task", "q", "lock the rancilio scale", run_task_q),
        # Takes the rancilio tool out of the group head, ends HOLDING the rancilio tool
        ("task", "s", "take the rancilio tool out of the group head", run_task_s),
        # Cleans on the silicone brush, ends HOLDING the rancilio tool
        ("task", "t", "clean on the silicone brush", run_task_t),
        # Cleans on the bristle brush, ends HOLDING the rancilio tool
        ("task", "u", "clean on the bristle brush", run_task_u),
        # Returns the rancilio tool to the stand, ends empty handed at the tool stand
        ("task", "v", "return the rancilio tool to the stand", run_task_v),
        # Carries the cup to the customer zone last, once the rancilio tool is out of the group head and back on the
        # stand, so the gripper is empty for the cup tool. Ends with the cup tool back on the stand
        ("task", "r", "carry the cup to the customer zone", run_task_r),
        # Sends the robot home, r finishes at the cup tool stand rather than at home_l
        ("waypoint", "home", "return to home_l", RDK.Item("Home_L", ITEM_TYPE_TARGET)),
    ]

    # Only a simulation run can be paused, space in the terminal pauses it and space again plays it. On the real
    # robot the run goes straight through
    contacts = []
    if RDK.RunMode() == RUNMODE_SIMULATE:
        if CHECK_COLLISIONS:
            contacts = enable_collision_report(UR5, tls)
        enable_space_pause(UR5, tls)

    for step, (kind, name, description, item) in enumerate(sequence):
        label = f"{kind} {name.upper()} ({step + 1} of {len(sequence)}): {description}"

        print(f"[main] Now running {label}")
        try:
            if kind == "task":
                item(UR5, tls, RDK)
            elif kind == "waypoint":
                UR5.MoveJ(item, True)
        except Exception as exc:
            print(f"[main] Failed during {label}, stopping the sequence here: {exc}")
            raise

    print("[main] Full run complete")
    if CHECK_COLLISIONS:
        print(f"[main] {len(contacts)} moves touched something")
        # A line of code that touched the same things more than once, such as a step in a loop, is listed once
        counts = {}
        for where, source, touching in contacts:
            line = f"{where}  {source}  ->  {', '.join(touching)}"
            counts[line] = counts.get(line, 0) + 1
        for line, count in counts.items():
            print(f"    {line}" + (f"  (x{count})" if count > 1 else ""))


if __name__ == "__main__":
    main()
