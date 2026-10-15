# _partP.py
# Task p, use the mazzer tool to operate the rancilio hot water switch until the scale reports 32 g of water
# The switch is a rocker (point 37), pressing its top half turns the water on and its bottom half turns it off,
# like part c's buttons. The rancilio scale is read while the water runs, like part d's dosing
from robodk.robolink import *
from robodk.robomath import *
from time import sleep, time
import tools
from _common import get_point, build_pose, rotate_about_tip, compute_waypoints, run_visual, enable_space_pause
from _partL import compute_rancilio_pose

# Ip address of the raspberry pi serving the rancilio scale on cart 3
IP_RANCILIO_3 = "192.168.22.4"

# Water set by the assignment brief, in grams dispensed into the cup
TARGET_G = 32.0
TOLERANCE_G = 0.1

# Time for the scale to settle before it is tared or read
SETTLE_SECONDS = 1


def compute_task_p_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, press_depth_mm=0):
    # Same steps as part a, on the rancilio's frame
    rancilio_pose = compute_rancilio_pose(fixture_fine_tune_mm)

    # The target is the switch rocker (point 37), with the tool pointing straight into the front of the machine
    point_local = get_point(37)
    axis_mapping = [
        [0, 0, 1],   # Tool x axis lines up with the rancilio's z axis, up
        [0, 1, 0],   # Tool y axis lines up with the rancilio's y axis
        [-1, 0, 0],  # Tool z axis lines up with the rancilio's negative x axis, into the front of the machine
    ]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = rancilio_pose * T_fixture_placement

    # The tool's contact point is its tip (point 51)
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    task_p_pose = T_world_target * T_tool_offset.inv()

    # Angle adjust about the tip
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_p_pose = rotate_about_tip(task_p_pose, tool_point_local, angle_adjust)

    # Pushes the tip press_depth_mm further in along the tool's own z, zero just touches the rocker
    task_p_pose = task_p_pose * transl(0, 0, press_depth_mm)

    return task_p_pose


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


def press_switch(UR5, approach_pose, press_pose):
    # Moves straight in from the approach pose to press the rocker, then straight back out again
    UR5.MoveL(approach_pose, True)
    UR5.MoveL(press_pose, True)
    UR5.MoveL(approach_pose, True)


def dispense_until_target(UR5, client, on_poses, off_poses, stop_early_g, poll_s, max_dispense_s):
    # Tares the scale with the empty cup on the pan, turns the water on, reads the scale until the cup is within
    # stop_early_g of the target, then turns the water off. Stops early after max_dispense_s so a broken scale
    # can't leave the water running. Every exit leaves the tool at the off approach pose
    sleep(SETTLE_SECONDS)
    client.tare()
    sleep(SETTLE_SECONDS)

    press_switch(UR5, *on_poses)
    UR5.MoveL(off_poses[0], True)

    start = time()
    while True:
        water_g = read_scale_grams(client)
        print(f"[task p] Dispensed {water_g:.1f} g")
        if water_g >= TARGET_G - stop_early_g:
            break
        if time() - start > max_dispense_s:
            print(f"[task p] WARNING, stopped after {max_dispense_s} s without reaching the target")
            break
        sleep(poll_s)

    press_switch(UR5, *off_poses)

    # Reads the scale again once the last drips have settled
    sleep(SETTLE_SECONDS)
    water_g = read_scale_grams(client)
    if water_g > TARGET_G + TOLERANCE_G:
        print(f"[task p] WARNING, overfilled, {water_g:.1f} g")
    elif water_g >= TARGET_G - TOLERANCE_G:
        print(f"[task p] Target water reached, {water_g:.1f} g")
    else:
        print(f"[task p] WARNING, underfilled, {water_g:.1f} g")
    return water_g


def run_task_p(UR5, tls, RDK, use_scale=None):
    """ Does not return the robot home, starts holding the mazzer tool where part o leaves it, and ends still holding
    it, backed off the switch, ready for part q """

    # ==================== FINE TUNING (edit these in the lab) ====================
    # The on and off presses are tuned separately

    # Shifts where the tip presses for ON, along the rancilio's own axes. z must put it on the top half of the rocker
    on_fixture_fine_tune_mm = (0, 0, 8)

    # The same for OFF, which must be on the bottom half of the rocker
    off_fixture_fine_tune_mm = (0, 0, -4)

    # Rotates the tool about its tip, for both presses
    approach_angle_adjust_deg = (0, 0, 0)

    # How far past touching the tip pushes in, for both presses, bigger if the rocker doesn't flip
    press_depth_mm = 5

    # How far back from the rocker the tip lines up before each press, and backs off to after
    approach_backoff_mm = 40

    # True reads the real scale and turns the water off at the target, False runs it for sim_dispense_s. This value
    # is only used when this file is run on its own, a full run takes USE_SCALE from __main.py instead
    if use_scale is None:
        use_scale = True

    # With the scale, turns the water off this many grams early, since it still runs while the rocker is being
    # pressed off. Tune it until the scale settles at 32 g
    stop_early_g = 8

    # With the scale, how often it is read while the water runs, in seconds
    poll_s = 0.1

    # With the scale, the longest the water is left running, in seconds, so a broken scale can't overflow the cup
    max_dispense_s = 120

    # Without the scale, how long the water runs, in seconds
    sim_dispense_s = 1

    # TARGET_G, TOLERANCE_G and SETTLE_SECONDS are at the top of this file
    # =============================================================================

    # Computes the poses pressing the rocker on and off
    on_pose = compute_task_p_pose(on_fixture_fine_tune_mm, approach_angle_adjust_deg, press_depth_mm)
    off_pose = compute_task_p_pose(off_fixture_fine_tune_mm, approach_angle_adjust_deg, press_depth_mm)

    # The pose lined up off the rocker before the off press, backed off along the tool's own z
    off_waypoints = compute_waypoints(off_pose, offsets=[
        ("approach", (0, 0, -(approach_backoff_mm + press_depth_mm)), (0, 0, 0)),
    ])
    # Jogged joint pose lined up off the top of the rocker, the arm swings round from part o's lever to here
    on_approach_pose = [9.129708, -83.365625, 136.423888, -233.058821, -39.167246, -89.999771]
    off_approach_pose = off_waypoints["approach"]

    # The mazzer tool is already held, carried over from task o
    # Swings round to the front of the rancilio and lines up off the top of the rocker
    UR5.MoveL(on_approach_pose, True)

    # Turns the water on, waits until the scale reports the target, or for a fixed time if no scale is in use,
    # then turns the water off
    if use_scale:
        client = connect_to_scale(IP_RANCILIO_3)
        dispense_until_target(UR5, client, (on_approach_pose, on_pose), (off_approach_pose, off_pose),
                              stop_early_g, poll_s, max_dispense_s)
    else:
        print("[task p] Scale not in use, running the water for a fixed time")
        press_switch(UR5, on_approach_pose, on_pose)
        UR5.MoveL(off_approach_pose, True)
        sleep(sim_dispense_s)
        press_switch(UR5, off_approach_pose, off_pose)

    # Ends lined up off the bottom of the rocker, still holding the mazzer tool for part q


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

    # Shows what tasks l and n leave behind, the rancilio tool in the group head, off the tool stand, and the cup on
    # the scale pan. Simulation only
    run_visual(RDK, "Show_Rancilio_Rancilio_Tool_Rotated")
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)
    run_visual(RDK, "Show_Rancilio_Scale_Cup")
    run_visual(RDK, "Show_Rancilio_Scale_Read")

    # Picks up the mazzer tool, in the full run it is already held, carried over from task o
    tls.mazzer_tool_attach_l_ati()

    run_task_p(UR5, tls, RDK)

    # Returns the mazzer tool and sends the robot home, in the full run task q carries on still holding it
    tls.mazzer_tool_detach_l_ati()
    UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)
