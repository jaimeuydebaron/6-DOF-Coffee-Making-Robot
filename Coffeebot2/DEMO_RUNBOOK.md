# Coffeebot Final Demo Runbook

Robot 3, station `Robot_3_2026`. The full run is `__main.py`, tasks a to v in the order below, starting and ending at `Home_L`.

---

## 1. Before the demo (lab computer)

- [ ] Copy these from the USB into **one folder**: `__main.py`, `__points.py`, `_common.py`, `tools.py`, and every `_part*.py` (A to V, plus GH). `__pycache__` is not needed.
- [ ] Open the `Robot_3_2026` station in RoboDK. The code finds items by name (`UR5`, `Home_L`, tool targets, `Show_…`/`Hide_…` programs), so it must be the same station.
- [ ] Check the scale package is installed in the Python you will run from. This should print nothing:
  ```
  python -c "import modbus_scale_client"
  ```
  If it errors, the run will stop at task d with the arm at the Mazzer lever.
- [ ] Both scale Pis are on and reachable:
  - Mazzer scale: `192.168.22.3` (task d)
  - Rancilio scale: `192.168.22.4` (task p)
- [ ] In RoboDK: **Connect** to the robot, then **Get position**.

## 2. Settings in `__main.py` (already set for the robot, check them)

| Setting | Should be |
|---|---|
| `RDK.setRunMode(RUNMODE_RUN_ROBOT)` | uncommented |
| `RDK.setRunMode(RUNMODE_SIMULATE)` | commented out |
| `run_visual(RDK, "Reset_Simulation_L")` | commented out |
| `USE_SCALE` | `True` |
| `CHECK_COLLISIONS` | `False` (ignored on the robot anyway) |

## 3. Starting the cell

- [ ] All three tools (rancilio, mazzer, cup) on the tool stand
- [ ] Rancilio tool basket clean and empty, Mazzer hopper has beans
- [ ] Cup dispenser loaded
- [ ] Mazzer scale and rancilio scale **locked**, nothing on either
- [ ] WDT shut
- [ ] Customer zone clear
- [ ] Robot clear of everything (it moves to `Home_L` first)
- [ ] Someone at the pendant stop the whole run

---

## 4. The run, task by task

**Joint / tool stand** first move = safe to start from anywhere clear.
**⚠ Straight line** first move = only safe if the arm is right where the previous task ended.

| # | Task | Starts holding | First move | What it does | Ends with (gripper, position, cell) |
|---|---|---|---|---|---|
| 1 | **a** | Nothing | Joint to `Home_L` | Picks the rancilio tool off the stand, sets it on the mazzer scale, lets go | **Empty**, beside the mazzer scale. Rancilio tool on the mazzer scale |
| 2 | **b** | Nothing | Tool stand (picks mazzer tool) | Unlocks the mazzer scale (slide lever, press) | **Mazzer tool**, staging pose by the Mazzer. Mazzer scale unlocked |
| 3 | **c** | Mazzer tool | Joint to staging | Presses Mazzer ON, grinds 15 s, presses OFF | **Mazzer tool**, staging pose |
| 4 | **d** | Mazzer tool | Joint to staging | Pulls the dose lever until the scale reads 20 g (max 40 pulls). *Needs mazzer scale Pi* | **Mazzer tool**, staging pose. Grounds in the rancilio tool |
| 5 | **e+m** | Mazzer tool | Joint to staging | Locks the mazzer scale, flicks the cup dispenser tab to drop a cup, returns the mazzer tool | **Empty**, at the mazzer tool stand. Mazzer scale locked, cup in the dispenser |
| 6 | **f** | Nothing | Joint to beside the mazzer scale | Grabs the rancilio tool off the mazzer scale, lifts it | **Rancilio tool**, above the mazzer scale |
| 7 | **g+h** | Rancilio tool | Joint to above the mazzer scale | Opens WDT, places the tool in it, lets go, shuts WDT | **Empty**, near staging. Rancilio tool in the WDT, shut |
| 8 | **i** | Nothing | Tool stand (picks mazzer tool) | Tip into the WDT rotor, 5 turns, lifts, returns the mazzer tool | **Empty**, at the mazzer tool stand |
| 9 | **j** | Nothing | Joint to near staging | Opens WDT, grabs the rancilio tool, lifts out, shuts WDT | **Rancilio tool**, beside the WDT |
| 10 | **k** | Rancilio tool | Joint towards the PUQ | Basket into the PUQ, holds 2 s while it tamps, backs out | **Rancilio tool**, in front of the PUQ |
| 11 | **l+n** | Rancilio tool | Joint towards the mazzer scale | Sets the rancilio tool on the mazzer scale. Cup tool: cup from dispenser to rancilio scale pan, cup tool back on stand. Picks the rancilio tool back up, into the group head, twists to lock, lets go | **Empty**, raised above/in front of the Rancilio. Tool locked in the group head, cup on the rancilio scale |
| 12 | **o** | Nothing | Tool stand (picks mazzer tool) | Unlocks the rancilio scale | **Mazzer tool**, backed off near the rancilio scale lever |
| 13 | **p** | Mazzer tool | ⚠ **Straight line** to in front of the water switch | Tares the scale, water ON, waits until ~22 g (max 120 s), water OFF. *Needs rancilio scale Pi* | **Mazzer tool**, just off the switch |
| 14 | **q** | Mazzer tool | ⚠ **Straight line** back to the rancilio scale lever | Locks the rancilio scale, returns the mazzer tool | **Empty**, at the mazzer tool stand. Rancilio scale locked |
| 15 | **s** | Nothing | Joint towards the Rancilio | Grabs the rancilio tool in the group head, untwists, drops it out, slides out the front | **Rancilio tool**, in front of the group head |
| 16 | **t** | Rancilio tool | ⚠ **Straight line** towards the tool cleaner | Presses the basket on the silicone brush 5 s | **Rancilio tool**, above the silicone brush |
| 17 | **u** | Rancilio tool | ⚠ **Straight line** to above the bristle brush | Presses the basket on the bristle brush 5 s | **Rancilio tool**, above the bristle brush |
| 18 | **v** | Rancilio tool | ⚠ **Straight line** pull back 150 mm | Returns the rancilio tool to the stand | **Empty**, at the rancilio tool stand |
| 19 | **r** | Nothing | Tool stand (picks cup tool) | Picks the cup off the rancilio scale, carries it to the customer zone, sets it down, returns the cup tool | **Empty**, at the cup tool stand. Cup in the customer zone |
| 20 | **home** | Nothing | Joint | Returns to `Home_L` | Done |

---

## 5. If a task fails

1. **Stop the robot** (pendant).
2. **Decide where to restart.** Normally the next task. The cell must match the **"Ends with"** column of the task *before* the one you restart at.
3. **Get the gripper right:**
   - Restarting at a task that **starts holding a tool**: make sure that tool is in the gripper.
   - Restarting at a task that **starts with nothing**: tool off, back on its stand, and **digital output 0 (tool attach) OFF** on the pendant, or the gripper won't latch the next tool.
4. **Get the cell right:** tools, cup, scale lock levers, WDT open/shut, as in the table.
5. **Jog the arm clear** of the machines.
   - If the restart task's first move is **⚠ Straight line** (p, q, t, u, v), jog it close to where the previous task ended, or restart further back instead (see below).
6. **In `__main.py`, comment out every entry in `sequence` above the restart task.** Leave the rest, including the final `home` waypoint.
7. Run `__main.py`.

### Safe restart points

These start empty handed with a joint or tool stand move, so they work from anywhere clear:

| Restart at | Cell must have |
|---|---|
| **a** | Everything as in section 3 |
| **b** | Rancilio tool on the mazzer scale. Mazzer tool on the stand |
| **f** | Rancilio tool on the mazzer scale (with grounds). Mazzer and cup tools on the stand. Cup in the dispenser |
| **i** | Rancilio tool in the WDT, WDT shut. Mazzer tool on the stand |
| **j** | Rancilio tool in the WDT. Mazzer tool on the stand |
| **o** | Rancilio tool locked in the group head. Cup on the rancilio scale. Mazzer tool on the stand. Rancilio scale locked |
| **s** | Rancilio tool locked in the group head. Mazzer and cup tools on the stand |
| **r** | Cup on the rancilio scale. All tools on the stand |

If a ⚠ straight line task needs restarting and the arm isn't where the previous task ended:
- **p or q** → restart at **o** (put the mazzer tool back on the stand, rancilio scale locked)
- **t, u or v** → either put the rancilio tool back locked in the group head and restart at **s**, or (skips the cleaning) put the rancilio tool on its stand by hand and restart at **r**

---

## 6. Watch closely (not testable in simulation)

| Task | What to watch | Why |
|---|---|---|
| **d** | Grams printed in the terminal each pull | Real scale code path, the sim ran it with the scale off |
| **i** | The first rotor turn, stop if the tool binds or jumps out | The sim's rotor can't turn, so the orbit (27.6 mm radius) is unproven |
| **p** | Grams printed every 0.1 s. **If the number stops climbing while water runs, press the rocker off or stop the robot** | If a scale read fails mid dispense, the code does **not** turn the water off. With `max_dispense_s = 120` a stuck reading can overfill the cup |
| **a, f, l** | The rancilio tool setting down on / lifting off the mazzer scale | The tool lands 4 mm low (`fixture_fine_tune_mm` z = -4), fine in sim but check it isn't pressing hard |

## 7. Simulation result (for reference)

Full run a to v in RoboDK simulation with collision checking: **all 20 steps completed, no unreachable poses, no singularity stops, no errors.** Every contact was an intended one (tool docking, levers, buttons, switch, fixtures, brushes, cup fork), except the task i rotor orbit, which is a simulation limitation.
