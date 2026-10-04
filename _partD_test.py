# _partD_test.py
# Test version of _partD.py, pulls the dose lever and releases it again and again until enter is pressed
# It reuses the poses and arc maths from _partD.py, so anything tuned there applies here too
import threading
from robodk.robolink import *
from robodk.robomath import *
import tools
from _partD import (
    compute_task_d_pull_steps, pull_once, return_to_start,
    approach_intermediate_pose_1, approach_intermediate_pose_2,
    approach_intermediate_pose_3, approach_intermediate_pose_4,
    lever_approach_pose, lever_off_intermediate_pose, retreat_intermediate_pose_1,
)


def start_stop_listener():
    # Waits for enter on a background thread, and returns an event that is set once it is pressed
    stop_requested = threading.Event()

    def wait_for_enter():
        input()
        stop_requested.set()

    threading.Thread(target=wait_for_enter, daemon=True).start()
    return stop_requested


def run_task_d_test(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================

    # pull_angle_deg
    # How far the tool sweeps the lever per pull, in degrees, about the world's z axis around the lever pivot.
    # Bigger = a longer pull, smaller = a shorter pull. Match this to _partD.py once it is dialled in.
    pull_angle_deg = 37

    # num_steps
    # How many intermediate points the arc is split into. More = smoother but slower, fewer = faster but jerkier.
    num_steps = 20

    # pivot_fine_tune_mm = (x, y, z) in mm
    # Shifts the centre the arc rotates about, along the mazzer's own local axes, so it changes the arc's radius.
    # Nudge it if the tool drifts away from, or pushes into, the lever partway through the pull.
    pivot_fine_tune_mm = (0, 0, 0)

    # attach_tool
    # True picks up and returns the mazzer tool from the stand like the real task. Set False if the tool is
    # already on the robot, so the test skips the pickup and starts straight from the staging poses.
    attach_tool = True

    # =============================================================================

    # Builds the pull arc from the pose the jogged pre pull joints correspond to
    lever_start_cartesian = UR5.SolveFK(lever_off_intermediate_pose)
    pull_steps = compute_task_d_pull_steps(lever_start_cartesian, pull_angle_deg, num_steps, pivot_fine_tune_mm)

    if attach_tool:
        tls.mazzer_tool_attach_l_ati()

    # Goes through the staging poses to clear the mazzer body
    UR5.MoveJ(approach_intermediate_pose_1, True)
    UR5.MoveJ(approach_intermediate_pose_2, True)
    UR5.MoveJ(approach_intermediate_pose_3, True)
    UR5.MoveJ(approach_intermediate_pose_4, True)

    # Moves in to the lever and stops at the pre pull pose
    UR5.MoveL(lever_approach_pose, True)
    UR5.MoveL(lever_off_intermediate_pose, True)

    # Pulls and releases the lever until enter is pressed
    # Enter is only checked between cycles, so the current pull and release always finish, and the tool
    # is back at the pre pull pose before it retreats
    print("[task d test] Pulling and releasing the lever, press enter to stop")
    stop_requested = start_stop_listener()
    cycle = 0
    while not stop_requested.is_set():
        cycle += 1
        print(f"[task d test] Cycle {cycle}, pulling")
        pull_once(UR5, pull_steps)
        print(f"[task d test] Cycle {cycle}, releasing")
        return_to_start(UR5, pull_steps, lever_off_intermediate_pose)
    print(f"[task d test] Stopped after {cycle} cycles")

    # Retreats from the pre pull pose back out through the staging poses
    UR5.MoveL(retreat_intermediate_pose_1, True)
    UR5.MoveJ(approach_intermediate_pose_4, True)
    UR5.MoveJ(approach_intermediate_pose_3, True)
    UR5.MoveJ(approach_intermediate_pose_2, True)
    UR5.MoveJ(approach_intermediate_pose_1, True)

    if attach_tool:
        tls.mazzer_tool_detach_l_ati()


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

    run_task_d_test(UR5, tls, RDK)
