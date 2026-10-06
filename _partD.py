# _partD.py
from robodk.robolink import *
from robodk.robomath import *
from time import sleep
import tools
from _common import get_point, build_pose, compute_mazzer_pose, compute_waypoints, run_visual, enable_space_pause

# Ip address of the raspberry pi serving the mazzer scale on cart 3
IP_MAZZER_3 = "192.168.22.3"

# Dose set by the assignment brief, in grams of grounds deposited in the rancilio tool
TARGET_G = 20.0
TOLERANCE_G = 0.1

# Time for the grounds to settle before the scale is read
SETTLE_SECONDS = 0


def rotate_about_point_world_z(pose, pivot_point_world, angle_deg):
    # Rotates a pose about the world z axis through a given world point
    to_pivot = transl(*pivot_point_world)
    rotation = rotz(angle_deg * pi/180)
    from_pivot = transl(*tuple(-c for c in pivot_point_world))
    return to_pivot * rotation * from_pivot * pose


def rotate_about_tip(pose, tip_offset_mm, angle_adjust):
    # Same as rotate_about_tip in _common.py
    return pose \
        * transl(*tip_offset_mm) \
        * angle_adjust \
        * transl(*tuple(-c for c in tip_offset_mm))


def compute_task_d_approach_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, dose_bar_mount_angle_deg):
    # Same steps as part a, on the mazzer's frame. Reference only, see run_task_d
    mazzer_pose = compute_mazzer_pose()
    mazzer_pose = mazzer_pose * transl(*fixture_fine_tune_mm)

    # The target is the dose lever face (point 24)
    point_local = get_point(24)
    axis_mapping = [[0, 1, 0], [0, 0, -1], [-1, 0, 0]]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = mazzer_pose * T_fixture_placement

    # The tool's contact point is the dose bar edge (point 52). It sits off the tool's centre axis, so the
    # tool's mounting angle is corrected for here
    tool_point_local = get_point(52)
    T_tool_offset = rotz(dose_bar_mount_angle_deg * pi/180) * transl(*tool_point_local)
    task_d_approach_pose = T_world_target * T_tool_offset.inv()

    # Angle adjust about the dose bar edge, so the contact point stays put
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_d_approach_pose = rotate_about_tip(task_d_approach_pose, tool_point_local, angle_adjust)

    return task_d_approach_pose


def compute_task_d_pull_steps(start_pose, pull_angle_deg, num_steps, pivot_fine_tune_mm):
    # The lever pivots about the mazzer's origin (point 21), so the pull is start_pose swept about the world z
    # through that point, sampled at num_steps angles
    mazzer_pivot_local = get_point(21)
    mazzer_pivot_local = tuple(mazzer_pivot_local[i] + pivot_fine_tune_mm[i] for i in range(3))

    mazzer_pose = compute_mazzer_pose()
    pivot_pose = mazzer_pose * transl(*mazzer_pivot_local)
    lever_pivot_world = tuple(pivot_pose[i, 3] for i in range(3))

    steps = []
    for i in range(1, num_steps + 1):
        # Negative because the sweep direction was found to be backwards otherwise
        angle = -pull_angle_deg * i / num_steps
        steps.append(rotate_about_point_world_z(start_pose, lever_pivot_world, angle))

    return steps


def check_arc_reachable(UR5, pull_steps):
    # Prints whether each point on the arc has an inverse kinematics solution
    for i, step_pose in enumerate(pull_steps, start=1):
        solution = UR5.SolveIK(step_pose)
        status = "reachable" if solution else "UNREACHABLE"
        print(f"[task d] Arc step {i} of {len(pull_steps)}, {status}")


def connect_to_scale(host):
    # Imported here so this file still runs in simulation on machines without the package installed
    from modbus_scale_client import modbus_scale_client
    return modbus_scale_client.ModbusScaleClient(host=host)


def read_scale_grams(client):
    # Reads the scale, and refuses to trust the reading unless the real
    # server and broker both answered, otherwise the client returns fake values
    weight = client.read()
    if not (client.server_exists() and client.broker_exists()):
        raise RuntimeError("Scale is not reporting real readings")
    return weight


def pull_once(UR5, pull_steps):
    # Sweeps the tool through the whole arc, ending at the pulled position
    for step_pose in pull_steps:
        UR5.MoveL(step_pose, True)


def return_to_start(UR5, pull_steps, start_pose):
    # Retraces the arc backwards so the lever is released along the same path it was pulled
    for step_pose in reversed(pull_steps[:-1]):
        UR5.MoveL(step_pose, True)
    UR5.MoveL(start_pose, True)


def dose_until_target(UR5, client, pull_steps, start_pose, max_pulls):
    # Tares the scale, then does a full pull, reads the scale and releases the lever, repeating until the
    # dose is in the target window. Every exit leaves the tool at the start pose
    sleep(SETTLE_SECONDS)
    client.tare()
    sleep(SETTLE_SECONDS)

    dosed_g = 0.0
    for pull_number in range(1, max_pulls + 1):
        pull_once(UR5, pull_steps)

        sleep(SETTLE_SECONDS)
        dosed_g = read_scale_grams(client)
        print(f"[task d] Pull {pull_number}, dosed {dosed_g:.1f} g")

        return_to_start(UR5, pull_steps, start_pose)

        if dosed_g >= TARGET_G - TOLERANCE_G:
            break

    if dosed_g > TARGET_G + TOLERANCE_G:
        print("[task d] WARNING, overdosed")
    elif dosed_g >= TARGET_G - TOLERANCE_G:
        print("[task d] Target dose reached")
    else:
        print(f"[task d] WARNING, target dose not reached after {max_pulls} pulls")
    return dosed_g

staging_intermediate_pose = [118.506308, -97.117079, 116.960032, -109.759306, -89.525329, -206.644986]

# Jogged joint waypoints that carry the tool around the mazzer body and in to the lever
approach_intermediate_pose_4 = [118.920000, -94.290000, 131.650000, -37.280000, -269.520000, -206.230000]

# Joint pose standing off from the lever, then the pre pull pose the sweep starts from
lever_approach_pose = [113.207042, -71.917966, 101.400920, -29.681113, -275.849172, -129.983301]
lever_off_intermediate_pose = [118.721162, -72.660959, 105.668068, -33.204839, -270.335260, -130.001963]

# Joint pose that clears the lever after the pull
retreat_intermediate_pose_1 = [129.931477, -78.302406, 113.186900, -35.167926, -224.125112, -130.206680]


def run_task_d(UR5, tls, RDK):
    # ==================== FINE TUNING (edit these in the lab) ====================
    # The pull sweeps the tool in an arc about the lever pivot, starting from the jogged lever_off_intermediate_pose

    # Degrees swept per pull, the main knob for the dose, bigger gives more grounds per pull
    pull_angle_deg = 37

    # Points the arc is split into, more is smoother but slower
    num_steps = 20

    # Shifts the centre of the arc along the mazzer's own axes, nudge it if the tool drifts off or digs into the lever
    pivot_fine_tune_mm = (0, 0, 0)

    # True reads the real scale and pulls until the target dose, False pulls once, for simulation
    use_scale = False

    # The most pulls allowed with the scale in use, so a broken scale can't make the robot pull forever
    max_pulls = 5

    # TARGET_G, TOLERANCE_G and SETTLE_SECONDS are at the top of this file

    # Reference only, these build task_d_approach_pose, which no longer drives any move. To move where the pull
    # starts, re-jog lever_off_intermediate_pose instead
    fixture_fine_tune_mm = (20, 40, -10)
    approach_angle_adjust_deg = (-90, 25, 140)
    dose_bar_mount_angle_deg = 0
    task_d_approach_pose = compute_task_d_approach_pose(
        fixture_fine_tune_mm, approach_angle_adjust_deg, dose_bar_mount_angle_deg)
    # ===========================================================================

    # The standoff was first computed like this, jogged near, then hardcoded as lever_approach_pose
    # waypoints = compute_waypoints(task_d_approach_pose, offsets=[("approach", (20, -50, 0), (0, 0, 0))])
    # UR5.MoveL(waypoints["approach"], True)

    # Builds the pull arc from the pose the jogged pre pull joints correspond to
    lever_start_cartesian = UR5.SolveFK(lever_off_intermediate_pose)
    pull_steps = compute_task_d_pull_steps(lever_start_cartesian, pull_angle_deg, num_steps, pivot_fine_tune_mm)


    # Run once with collision detection on to catch an unreachable point partway through the sweep
    # check_arc_reachable(UR5, pull_steps)

    # The mazzer tool is already held, carried over from task c

    UR5.MoveJ(staging_intermediate_pose, True)

    # Goes through the staging poses to clear the mazzer body
    UR5.MoveJ(approach_intermediate_pose_4, True)

    # Moves in to the lever and stops at the pre pull pose
    UR5.MoveL(lever_approach_pose, True)
    UR5.MoveL(lever_off_intermediate_pose, True)

    # Pulls the lever until the scale reports the target dose, or once if no scale is in use
    if use_scale:
        client = connect_to_scale(IP_MAZZER_3)
        dose_until_target(UR5, client, pull_steps, lever_off_intermediate_pose, max_pulls)
    else:
        print("[task d] Scale not in use, pulling the lever once")
        pull_once(UR5, pull_steps)

    # Retreats from the lever back out through the staging poses
    UR5.MoveL(retreat_intermediate_pose_1, True)
    UR5.MoveJ(approach_intermediate_pose_4, True)

    UR5.MoveJ(staging_intermediate_pose, True)

    # Ends still holding the mazzer tool, for task e


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

    # Shows the rancilio tool sitting on the mazzer scale and takes it off the tool stand, as task a leaves it, with
    # the scale unlocked, as task b leaves it. Simulation only
    run_visual(RDK, "Show_Mazzer_Scale_Rancilio_Tool")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)
    run_visual(RDK, "Show_Mazzer_Scale_Read")

    # Picks up the mazzer tool, in the full run it is already held, carried over from task c
    tls.mazzer_tool_attach_l_ati()

    run_task_d(UR5, tls, RDK)

    # Returns the mazzer tool and sends the robot home, in the full run task e carries on still holding it
    tls.mazzer_tool_detach_l_ati()
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
