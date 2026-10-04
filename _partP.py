# _partP.py
# Task p, use the mazzer tool to operate the rancilio hot water switch until the scale reports 32 g of water
# Built the same way as part d, which doses the grounds against the mazzer scale. The switch is a rocker on the
# front of the rancilio (point 37, the rocker in its lowered position). The tool points straight into the front of
# the machine, its tip pushes the rocker down from above to turn the water on, the rancilio scale is read until the
# cup holds the target, then the tip pushes the rocker back up from below to turn it off
from robodk.robolink import *
from robodk.robomath import *
from time import sleep, time
import tools
from _common import get_point, build_pose, solve_joints_near
from _partLlefthand import compute_rancilio_pose
from _partN import run_visual
from _partO import LEVER_CONFIG_HINT

# Ip address of the raspberry pi serving the rancilio scale on cart 3
IP_RANCILIO_3 = "192.168.22.4"

# Water set by the assignment brief, in grams dispensed into the cup
TARGET_G = 32.0
TOLERANCE_G = 0.1

# Time for the scale to settle before it is tared or read
SETTLE_SECONDS = 1


def compute_task_p_pose(fixture_fine_tune_mm, tilt_deg, offset_mm=(0, 0, 0)):
    rancilio_pose = compute_rancilio_pose(fixture_fine_tune_mm)

    # Sets the placement point as the rocker in its lowered position, moved along the rancilio's own axes by
    # offset_mm, so the tip can start above or below it and push it along z
    point_local = get_point(37)

    # Tool orientation found by inspection, the tool pointing straight into the front of the machine
    axis_mapping = [
        [0, 0, 1],   # Tool x axis lines up with the rancilio's z axis, up
        [0, 1, 0],   # Tool y axis lines up with the rancilio's y axis
        [-1, 0, 0],  # Tool z axis lines up with the rancilio's negative x axis, into the front of the machine
    ]
    T_into_front = build_pose(axis_mapping, (0, 0, 0))

    # Tilts the tool about the tip, positive points the tip further down onto the rocker
    T_fixture_placement = transl(*point_local) * transl(*offset_mm) * roty(-tilt_deg * pi/180) * T_into_front
    T_world_target = rancilio_pose * T_fixture_placement

    # Sets the mazzer tool's contact point, the cushioning cylinder tip (point 51)
    tool_point_local = get_point(51)
    T_tool_offset = transl(*tool_point_local)
    return T_world_target * T_tool_offset.inv()


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


def push_switch(UR5, start_pose, end_pose, backoff_mm):
    # Lines up off the rocker, touches it, pushes it across, then backs straight off again
    UR5.MoveL(start_pose * transl(0, 0, -backoff_mm), True)
    UR5.MoveL(start_pose, True)
    UR5.MoveL(end_pose, True)
    UR5.MoveL(end_pose * transl(0, 0, -backoff_mm), True)


def wait_for_target(client, stop_early_g, poll_s, max_dispense_s):
    # Reads the scale while the water runs, until it is within stop_early_g of the target, or the time limit is
    # hit so a broken scale can't leave the water running
    start = time()
    while True:
        water_g = read_scale_grams(client)
        print(f"[task p] {water_g:.1f} g")
        if water_g >= TARGET_G - stop_early_g:
            return water_g
        if time() - start > max_dispense_s:
            print(f"[task p] WARNING, stopped after {max_dispense_s} s without reaching the target")
            return water_g
        sleep(poll_s)


def run_task_p(UR5, tls, RDK):
    """ Starts holding the mazzer tool backed off the rancilio scale lever where part o ends, and ends holding it
    backed off the switch, ready for part q """

    # ==================== FINE TUNING (edit these in the lab) ====================
    fixture_fine_tune_mm = (0, 0, 0)  # moves every pose, along the rancilio's axes, x out the front, z up
    tilt_deg = 0                      # tilts the tool down about its tip, 0 is level, straight into the machine

    # Where the tip starts and stops for each push, as offsets from the lowered rocker (point 37) along the
    # rancilio's axes, z is up. On pushes down from above the rocker, off pushes back up from below it
    on_start_mm = (0, 0, 15)    # touches the top of the rocker
    on_end_mm = (0, 0, 0)       # pushes it down to the lowered position, water on
    off_start_mm = (0, 0, -15)  # touches the bottom of the rocker
    off_end_mm = (0, 0, 10)     # pushes it back up past level, water off

    backoff_mm = 60       # how far back along the tool the tip lines up before each push, and backs off to after
    lift_off_mm = 100     # how far straight up the tool lifts off the scale lever first, clear of the rancilio
                          # tool's handle, before swinging across to the switch

    # use_scale
    # True on the lab network so the water is checked against the real scale and the switch is turned off at the
    # target. False runs the water for sim_dispense_s with no scale reading, for simulation.
    use_scale = True

    # stop_early_g
    # Only matters when use_scale is True. Turns the water off this many grams before the target, since water
    # still runs while the tool moves round to the switch and the rocker turns off. Tune it in the lab until the
    # scale settles at 32 g (+/- 0.1 g).
    stop_early_g = 0.0

    poll_s = 0.1           # how often the scale is read while the water runs
    max_dispense_s = 30    # the longest the water is left running, so a broken scale can't overflow the cup
    sim_dispense_s = 5     # how long the water runs when use_scale is False

    # The target water (TARGET_G), the tolerance (TOLERANCE_G) and the wait for the scale to settle (SETTLE_SECONDS)
    # are constants at the top of this file, not here.
    # =============================================================================

    def switch(offset_mm):
        return compute_task_p_pose(fixture_fine_tune_mm, tilt_deg, offset_mm)

    on_start_pose = switch(on_start_mm)
    on_end_pose = switch(on_end_mm)
    off_start_pose = switch(off_start_mm)
    off_end_pose = switch(off_end_mm)

    # Makes sure the poses below are read in the robot base frame with the master tool, as the assignment requires
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))

    # Connects to the scale and tares it with the empty cup on the pan, before any water runs
    if use_scale:
        client = connect_to_scale(IP_RANCILIO_3)
        sleep(SETTLE_SECONDS)
        client.tare()
        sleep(SETTLE_SECONDS)

    # Lifts straight up off the scale lever, clear of the rancilio tool's handle, then swings across to line up
    # with the rocker, still in the right hand configuration
    UR5.MoveL(transl(0, 0, lift_off_mm) * UR5.Pose(), True)
    UR5.MoveJ(solve_joints_near(UR5, on_start_pose * transl(0, 0, -backoff_mm), LEVER_CONFIG_HINT), True)

    # Pushes the rocker down to turn the water on, then lines up below it ready to turn it off
    push_switch(UR5, on_start_pose, on_end_pose, backoff_mm)
    UR5.MoveL(off_start_pose * transl(0, 0, -backoff_mm), True)

    # Waits until the scale reports the target, or for a fixed time if no scale is in use
    if use_scale:
        wait_for_target(client, stop_early_g, poll_s, max_dispense_s)
    else:
        print("[task p] Scale not in use, running the water for a fixed time")
        sleep(sim_dispense_s)

    # Pushes the rocker back up to turn the water off, and backs off, still holding the mazzer tool for part q
    push_switch(UR5, off_start_pose, off_end_pose, backoff_mm)

    # Reports the final reading once the last drips have settled
    if use_scale:
        sleep(SETTLE_SECONDS)
        water_g = read_scale_grams(client)
        if water_g > TARGET_G + TOLERANCE_G:
            print(f"[task p] WARNING, overfilled, {water_g:.1f} g")
        elif water_g >= TARGET_G - TOLERANCE_G:
            print(f"[task p] Target reached, {water_g:.1f} g")
        else:
            print(f"[task p] WARNING, underfilled, {water_g:.1f} g")


if __name__ == "__main__":
    RDK = Robolink()

    # Simulation settings, use when testing in robodk only
    RDK.setRunMode(RUNMODE_SIMULATE)

    # Real robot settings, uncomment when running on the physical robot
    # RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Connect to the robot before running, in the connect panel choose connect robot then press connect and get position

    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
    tls = tools.Tools(RDK)

    # Resets the simulation, then shows what parts l and n leave behind, the rancilio tool in the group head and
    # the cup on the scale pan. Simulation only, comment these out on the real robot
    run_visual(RDK, "Reset_Simulation_R")
    run_visual(RDK, "Show_Rancilio_Rancilio_Tool_Rotated")
    run_visual(RDK, "Show_Rancilio_Scale_Cup")

    # Part p carries on from where part o ends, holding the mazzer tool, so both run back to back
    from _partO import run_task_o
    run_task_o(UR5, tls, RDK)
    run_task_p(UR5, tls, RDK)
