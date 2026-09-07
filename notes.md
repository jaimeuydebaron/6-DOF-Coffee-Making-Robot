# Notes for how RDK works

- The `.rdk` file is a **station** — the whole simulation environment set up by Rodney and Michael.
- The robot, table, coffee machine, etc. are all **items** in the station, organized in a **tree**, like a file explorer's folder structure.
- The **station tree** is the panel on the left in the RoboDK UI.
- **Item types** are tags on each item, e.g.:
  - `ITEM_TYPE_ROBOT` — the robot arm
  - `ITEM_TYPE_PROGRAM` — a prebuilt sequence of actions (e.g. "Reset Simulation")
  - `ITEM_TYPE_TARGET` — a saved position/pose

- `RDK.Item("UR5", ITEM_TYPE_ROBOT)` means: go into the tree, find the item named "UR5", confirm it's a robot, and return a handle to it.
  - This handle gets stored in a variable (e.g. `UR5 = RDK.Item(...)`), so you can reference and control that item throughout the rest of the code.

- A **Tool Program** is a button-press macro sitting in the station tree.
  - Triggered by right-clicking it in the RoboDK UI.
  - Used for manual testing/demoing by clicking, without writing code.
  - Not meant to be called from your own Python script.

- The `tools.py` file defines a `Tools` class, which bundles a set of functions for real-world actuation.
  - e.g. `tls.cup_tool_attach_r_ati()` → attach the cup tool
  - e.g. `tls.cup_tool_open_ur5()` → open the cup tool's gripper

- What we are building ourselves: the physical transformations (poses) that move the tool head from one place to another.

## Coffee-making equipment (stations on the table)

| Item | What it actually is | What the robot does with it |
|---|---|---|
| **Mazzer** | The coffee grinder | Turn on/off (15s grind), pull dosing lever to deposit ground coffee (20±0.1g), operated using the Mazzer tool |
| **Mazzer Scale** | Weighing scale under the Mazzer | Lock/unlock, reads grind weight |
| **Rancilio** | The espresso machine | Insert Rancilio tool (portafilter) into group head, flip switch, dispense hot water (32±0.1g) into cup |
| **Rancilio Scale** | Weighing scale under the Rancilio's water outlet | Lock/unlock, measures water dispensed |
| **WDT (fixture)** | Weiss Distribution Technique fixture — mechanically redistributes/stirs loose grounds before tamping | Open, insert Rancilio tool, spin rotor 5 full revolutions, remove, close |
| **PUQ (fixture)** | Tamping station — compacts grounds into a puck before brewing | Insert Rancilio tool, wait 2s while it tamps automatically |
| **Cup Dispenser** | Stack of empty cups with a mechanical dispensing gate | Open/close gate, dispenses one cup at a time |
| **Rancilio Tool Cleaner** | Cleaning station with a bristle brush and a silicone brush | Press used Rancilio tool against each brush for 5s |
| **Tool Stand** | Home base for picking up/returning tools between tasks | Robot starts/ends there with Rancilio and Mazzer tools when not in use |
| **Customer zone** | Defined area on the table | Final placement location for the finished cup |

## Interchangeable robot tools (mounted on the UR5's wrist)

| Tool | What it's for |
|---|---|
| **Rancilio tool** | The portafilter — holds coffee grounds, moved between grinder, WDT, tamper, and espresso machine group head |
| **Mazzer tool** | General actuator tool — presses buttons, flips switches, pulls levers, turns WDT rotor |
| **Cup tool** | Gripper for picking up and placing the cup |

**Constraint:** robot and tool must stay within the bounding box defined by the table's physical edges at all times.

## Demonstration tasks (a–v)

**Milestone (due 5pm Friday week 3): tasks (b), (c), (m)**

1. (a) Pick up the Rancilio tool and place it on the Mazzer Scale pan.
2. **(b) Use the Mazzer tool to unlock the Mazzer Scale.**
3. **(c) Use the Mazzer tool to turn the Mazzer on, wait 15s, and turn the Mazzer off.**
4. (d) Use the Mazzer tool to pull the Mazzer dosing lever until the scale reports 20±0.1g of coffee grounds has been deposited in the Rancilio tool.
5. (e) Use the Mazzer tool to lock the Mazzer Scale.
6. (f) Remove the Rancilio tool from the Mazzer.
7. (g) Open the WDT fixture, and place the Rancilio tool into the WDT fixture.
8. (h) Release the Rancilio tool and close the WDT fixture.
9. (i) Use the Mazzer tool to turn the WDT rotor five full revolutions.
10. (j) Open the WDT fixture, remove the Rancilio tool and close the WDT fixture.
11. (k) Place the Rancilio tool into the PUQ fixture, and wait 2 seconds while the machine tamps the coffee grounds.
12. (l) Remove the Rancilio tool from the PUQ fixture, and insert it into the Rancilio group head.
13. **(m) Use the Mazzer tool to operate the cup dispenser.**
14. (n) Use the cup tool to pick up the dispensed cup, and place it on the Rancilio Scale pan.
15. (o) Use the Mazzer tool to unlock the Rancilio Scale.
16. (p) Use the Mazzer tool to operate the Rancilio hot water switch until the scale reports 32±0.1g of water has been dispensed in the cup.
17. (q) Use the Mazzer tool to lock the Rancilio Scale.
18. (r) Use the cup tool to carefully pick up the cup of coffee and place it in the customer zone.
19. (s) Remove the Rancilio tool from the group head.
20. (t) Position the Rancilio tool over the Rancilio Tool Cleaner fixture silicone brush, and actuate for 5s.
21. (u) Position the Rancilio tool over the Rancilio Tool Cleaner fixture bristle brush, and actuate for 5s.
22. (v) Return the Rancilio tool to the tool stand.

**Note:** robot must start and end in the (left or right) Home position.

**Marking:** 1 mark per task correctly demonstrated. Collisions: -1 each, max -3. Out-of-bounding-box: -1 flat. Cannot score below 0 overall.