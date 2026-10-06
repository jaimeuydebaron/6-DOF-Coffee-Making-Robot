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

| # | Task | Assignee | Status |
|---|------|----------|--------|
| 1 | (a) Pick up the Rancilio tool and place it on the Mazzer Scale pan. | Jaime | Done, TESTED WITH PART B |
| 2 | **(b) Use the Mazzer tool to unlock the Mazzer Scale.** | Jaime | Done, TESTED WITH PART A|
| 3 | **(c) Use the Mazzer tool to turn the Mazzer on, wait 15s, and turn the Mazzer off.** | Jaime | Done |
| 4 | (d) Use the Mazzer tool to pull the Mazzer dosing lever until the scale reports 20±0.1g of coffee grounds has been deposited in the Rancilio tool. | Jaime | Done, untested |
| 5 | (e) Use the Mazzer tool to lock the Mazzer Scale. | Jaime | Done |
| 6 | (f) Remove the Rancilio tool from the Mazzer. | Jaime | Not started |
| 7 | (g) Open the WDT fixture, and place the Rancilio tool into the WDT fixture. | Jaime | Done untested |
| 8 | (h) Release the Rancilio tool and close the WDT fixture. | Jaime | Done |
| 9 | (i) Use the Mazzer tool to turn the WDT rotor five full revolutions. | Jaime | Done, untested |
| 10 | (j) Open the WDT fixture, remove the Rancilio tool and close the WDT fixture. | Jaime | Not started |
| 11 | (k) Place the Rancilio tool into the PUQ fixture, and wait 2 seconds while the machine tamps the coffee grounds. | Jaime | Not started |
| 12 | (l) Remove the Rancilio tool from the PUQ fixture, and insert it into the Rancilio group head. | Jaime | Not started |
| 13 | **(m) Use the Mazzer tool to operate the cup dispenser.** | Jaime | Done |
| 14 | (n) Use the cup tool to pick up the dispensed cup, and place it on the Rancilio Scale pan. | Johann | Not started |
| 15 | (o) Use the Mazzer tool to unlock the Rancilio Scale. | Johann | Not started |
| 16 | (p) Use the Mazzer tool to operate the Rancilio hot water switch until the scale reports 32±0.1g of water has been dispensed in the cup. | Johann | Not started |
| 17 | (q) Use the Mazzer tool to lock the Rancilio Scale. | Johann | Not started |
| 18 | (r) Use the cup tool to carefully pick up the cup of coffee and place it in the customer zone. | Josh | Not started |
| 19 | (s) Remove the Rancilio tool from the group head. | Josh | Not started |
| 20 | (t) Position the Rancilio tool over the Rancilio Tool Cleaner fixture silicone brush, and actuate for 5s. | Josh | Not started |
| 21 | (u) Position the Rancilio tool over the Rancilio Tool Cleaner fixture bristle brush, and actuate for 5s. | Josh | Not started |
| 22 | (v) Return the Rancilio tool to the tool stand. | Josh | Not started |

**Note:** robot must start and end in the (left or right) Home position.

**Marking:** 1 mark per task correctly demonstrated. Collisions: -1 each, max -3. Out-of-bounding-box: -1 flat. Cannot score below 0 overall.

## Task order analysis (a–v)

**Fixture clusters, by world-frame x-position:**

| Right cluster (x: 380–590) | Left cluster (x: -150 to -680) |
|---|---|
| Mazzer (504, -420, 320) | Rancilio (-411, -480, 348) |
| Mazzer Scale (441, -274, 42) | Rancilio Scale (-386, -330, 42) |
| WDT (590, -99, 86) | Cup Dispenser (-589, -222, 213) |
| PUQ (385, 84, 271) - boundary, standalone on +y | Tool Cleaner (-151, -549, 179) |
| | Customer Zone (-420, 150, 0) |

Two clean clusters, right and left, with PUQ sitting at the boundary between them.

**Tool-hold segments in the given a-v order:**

1. Rancilio - (a)
2. Mazzer - (b, c, d, e)
3. Rancilio - (f, g, h)
4. Mazzer - (i)
5. Rancilio - (j, k, l)
6. Mazzer - (m)
7. Cup - (n)
8. Mazzer - (o, p, q)
9. Cup - (r)
10. Rancilio - (s, t, u, v)

9 tool changes total.

**Why the given order is already close to optimal:**

1. Causal constraints fix most of the sequence regardless of efficiency - grind before distribute, distribute before tamp, tamp before brew, brew before serve. No reordering freedom here.
2. The tool-change count is close to the minimum achievable - every switch happens because the task genuinely needs a different tool. Can't hold two tools at once, so alternating is unavoidable whenever an actuation step sits between transport steps (e.g. task i needs the Mazzer tool mid-way through the Rancilio tool's j-k-l carry).
3. The physical path is already a single right-to-left sweep, with only one visit to the PUQ boundary zone (task k) between the two clusters - no backtracking, no re-crossing the table.

**Theoretical (not practical) freedom:**
- Task (e) locking the Mazzer Scale doesn't have to happen immediately before (f), but leaving it unlocked risks the lever fouling the tool on removal - keeping it there is the safer mechanical choice, not just procedural.
- Task (m) dropping the cup could technically happen earlier since it's independent of grinding progress, but it's already adjacent to the Rancilio cluster it needs to be near for (n) - moving it earlier adds no efficiency and only risks the empty cup sitting exposed longer.

**Recommendation:** keep the assignment's given a-v order. It reflects both the causal requirement (grind -> distribute -> tamp -> brew -> serve) and a spatially efficient single sweep across the cart. Worth stating explicitly in the report's time-and-motion section - shows the sequence wasn't arbitrary and justifies why further reordering wouldn't reduce cycle time.