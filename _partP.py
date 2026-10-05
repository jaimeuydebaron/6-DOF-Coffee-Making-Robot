# _partP.py
# Task p, use the mazzer tool to operate the rancilio hot water switch until the scale reports 32 g of water
# The hot water switch is a rocker on the front of the rancilio (point 37). Pressing the top half of the rocker into the
# machine turns the water on, pressing the bottom half turns it off, the same way part c presses the mazzer's on and
# off buttons. While the water runs the rancilio scale is read, the same way part d reads the mazzer scale
from robodk.robolink import *
from robodk.robomath import *
from time import sleep, time
import tools
from _common import get_point, build_pose, rotate_about_tip, compute_waypoints
from _partL import compute_rancilio_pose

# Ip address of the raspberry pi serving the rancilio scale on cart 3
IP_RANCILIO_3 = "192.168.22.4"

# Water set by the assignment brief, in grams dispensed into the cup
TARGET_G = 32.0
TOLERANCE_G = 0.1

# Time for the scale to settle before it is tared or read
SETTLE_SECONDS = 1


def compute_task_p_pose(fixture_fine_tune_mm, approach_angle_adjust_deg, press_depth_mm=0):
    rancilio_pose = compute_rancilio_pose(fixture_fine_tune_mm)

    # Sets the placement point as the switch rocker, with the tool orientation found by inspection, the tool
    # pointing straight into the front face of the machine
    point_local = get_point(37)
    axis_mapping = [
        [0, 0, 1],   # Tool x axis lines up with the rancilio's z axis, up
        [0, 1, 0],   # Tool y axis lines up with the rancilio's y axis
        [-1, 0, 0],  # Tool z axis lines up with the rancilio's negative x axis, into the front of the machine
    ]
    T_fixture_placement = build_pose(axis_mapping, point_local)
    T_world_target = rancilio_pose * T_fixture_placement

    # Sets the mazzer tool's contact point, the cushioning cylinder tip (point 51)
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    task_p_pose = T_world_target * T_tool_offset.inv()

    # Applies the approach angle correction, rotated about the tool tip so its position doesn't move
    angle_adjust = rotx(approach_angle_adjust_deg[0]*pi/180) \
                 * roty(approach_angle_adjust_deg[1]*pi/180) \
                 * rotz(approach_angle_adjust_deg[2]*pi/180)
    task_p_pose = rotate_about_tip(task_p_pose, tool_point_local, angle_adjust)

    # Pushes the tip press_depth_mm further in along the tool's own z, into the rocker, zero just touches it
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


def run_task_p(UR5, tls, RDK):
    """ Does not return the robot home, starts holding the mazzer tool where part o leaves it, and ends still holding
    it, backed off the switch, ready for part q """

    # ==================== FINE TUNING (edit these in the lab) ====================
    # The on press and the off press are tuned separately, so changing one does not affect the other.

    # on_fixture_fine_tune_mm = (x, y, z) in mm
    # Moves WHERE the tip presses to turn the water ON, by shifting the assumed position of the rancilio along the
    # rancilio's own axes. The tool keeps its angle and just moves as a whole.
    #   x: shifts along the rancilio's local x, out the front of the machine
    #   y: shifts along the rancilio's local y, across the front of the machine
    #   z: raises (+) or lowers (-) the press, it must be on the top half of the rocker to turn the water on
    on_fixture_fine_tune_mm = (0, 0, 8)

    # off_fixture_fine_tune_mm = (x, y, z) in mm
    # Same as on_fixture_fine_tune_mm, but for the press that turns the water OFF, which must be on the bottom half
    # of the rocker.
    off_fixture_fine_tune_mm = (0, 0, -4)

    # approach_angle_adjust_deg = (rx, ry, rz) in degrees
    # Rotates the tool about its own x, y, z axes (x, then y, then z) for both presses. It pivots about the tool tip
    # (point 51), so the tip stays on the rocker and only the tool's facing direction changes. Use it if the tool body
    # gets close to the rancilio tool's handle or the cup.
    approach_angle_adjust_deg = (0, 0, 0)

    # press_depth_mm
    # How far past touching the rocker the tip pushes into the machine, along the tool's own z, for both presses.
    # Bigger = a firmer press, make it bigger if the rocker doesn't flip, smaller if the tool pushes too hard.
    press_depth_mm = 5

    # approach_backoff_mm
    # How far back from the rocker, along the tool's own z, the tip lines up before each press and backs off to after.
    # Bigger = more clearance from the front of the machine, smaller = shorter presses.
    approach_backoff_mm = 40

    # use_scale
    # True on the lab network so the water is checked against the real scale and turned off at the target.
    # False runs the water for sim_dispense_s with no scale reading, for simulation.
    use_scale = False

    # stop_early_g
    # Only matters when use_scale is True. Turns the water off this many grams before the target, since water still
    # runs while the tool presses the rocker off. Tune it in the lab until the scale settles at 32 g (+/- 0.1 g).
    stop_early_g = 0.0

    # poll_s
    # Only matters when use_scale is True. How often the scale is read while the water runs, in seconds.
    poll_s = 0.1

    # max_dispense_s
    # Only matters when use_scale is True. The longest the water is left running, in seconds, so a broken scale
    # can't overflow the cup.
    max_dispense_s = 30

    # sim_dispense_s
    # Only matters when use_scale is False. How long the water runs, in seconds.
    sim_dispense_s = 1

    # The target water (TARGET_G), the tolerance (TOLERANCE_G) and the wait for the scale to settle (SETTLE_SECONDS)
    # are constants at the top of this file, not here.
    # on_approach_pose below is a jogged joint pose and is NOT affected by the settings here. Re-jog it if the arm
    # hits the rancilio tool's handle or the cup on the way round to the front of the rancilio.
    # =============================================================================

    # Computes the poses pressing the rocker on and off
    on_pose = compute_task_p_pose(on_fixture_fine_tune_mm, approach_angle_adjust_deg, press_depth_mm)
    off_pose = compute_task_p_pose(off_fixture_fine_tune_mm, approach_angle_adjust_deg, press_depth_mm)

    # Computes the poses lined up off the rocker before each press, backed straight off along the tool's own z
    # Each offset is (name, (x, y, z) in mm, (rx, ry, rz) in degrees), applied in the tool's OWN frame
    on_waypoints = compute_waypoints(on_pose, offsets=[
        ("approach", (0, 0, -(approach_backoff_mm + press_depth_mm)), (0, 0, 0)),
    ])
    off_waypoints = compute_waypoints(off_pose, offsets=[
        ("approach", (0, 0, -(approach_backoff_mm + press_depth_mm)), (0, 0, 0)),
    ])
    # Jogged joint pose lined up off the top of the rocker, the arm swings round from part o's lever to here
    on_approach_pose = [13.947107, -97.104966, 115.451384, -18.346418, 43.984643, 90.000000]
    off_approach_pose = off_waypoints["approach"]

    # The mazzer tool is already held, carried over from task o
    # Swings round to the front of the rancilio and lines up off the top of the rocker
    UR5.MoveJ(on_approach_pose, True)

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

    # Sends the robot home
    # UR5.MoveJ(RDK.Item("Home_L", ITEM_TYPE_TARGET), True)


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

    run_task_p(UR5, tls, RDK)
