# __main.py
# Runs the completed tasks back to back in one continuous simulation or robot run
from robodk.robolink import *
from robodk.robomath import *
import tools

from _partA import run_task_a
from _partB import run_task_b
from _partC import run_task_c
from _partD import run_task_d
from _partE import run_task_e
from _partF import run_task_f
from _partGH import run_task_g
from _partM import run_task_m


# transition waypoints — clear a specific task-to-task handoff, not part
# of either task's own internal logic. name them by the transition they
# serve so it's obvious why each one exists.
# example: e_to_f_transition_joints = [j1, j2, j3, j4, j5, j6]


def main():
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)

    # Resets once for the whole sequence, the left hand variant since task a is the first task and runs left handed
    RDK.Item("Reset_Simulation_L", ITEM_TYPE_PROGRAM).RunCode()

    # Each entry is either ("task", name, run_task_x) or ("waypoint", name, joints_or_pose).
    # Waypoint entries slot between tasks wherever a transition needs a specific
    # move that isn't part of either task's own logic.
    sequence = [
        ("task", "a", run_task_a),
        ("task", "b", run_task_b),
        ("task", "c", run_task_c),
        # Task d, dosing the grind weight against the scale reading, is not implemented yet
        ("task", "d", run_task_d),
        # FIXME, task e currently finishes without returning the mazzer tool to its stand, but
        # task f assumes the robot is empty handed so it can pick up the rancilio tool instead,
        # this needs a fix in _partE.py's run_task_e before this sequence can run cleanly
        ("task", "e", run_task_e),

        # example: a transition waypoint between e and f
        # ("waypoint", "e_to_f", e_to_f_transition_joints),

        # FIXME, task f swaps the RDK ghost and attached visuals to show the rancilio tool being
        # picked up, but never actuates the real gripper through tls.student_tool_attach(), this
        # needs a fix in _partF.py's run_task_f before this will work on the physical robot
        ("task", "f", run_task_f),

        ("task", "g", run_task_g),
        # Tasks h, i, j, k and l are not implemented yet
        ("task", "m", run_task_m),
    ]

    for kind, name, item in sequence:
        try:
            if kind == "task":
                item(UR5, tls, RDK)
            elif kind == "waypoint":
                UR5.MoveJ(item, True)
        except Exception as exc:
            print(f"Step {name} failed, stopping the sequence here: {exc}")
            raise


if __name__ == "__main__":
    main()