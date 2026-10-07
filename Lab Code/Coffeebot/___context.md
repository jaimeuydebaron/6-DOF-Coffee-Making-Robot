# CoffeeBot Assignment — Context for AI Chat

Paste this whole file into a new chat before asking for help with ENMT482's UR5 coffee-cart assignment. It gives the AI the background it needs so you don't have to re-explain from scratch.

## The assignment

ENMT482 Manipulators Assignment (25% of course grade). Goal: program a UR5 robot arm (in RoboDK, then on the real robot) to make a cup of coffee using interchangeable tools (Rancilio portafilter, Mazzer actuator tool, cup gripper) on a coffee cart with a grinder (Mazzer), espresso machine (Rancilio), a WDT distribution fixture, a PUQ tamper, a cup dispenser, scales, and a tool cleaner.

Tasks are lettered (a) through (v), covering the full coffee-making sequence: dose grounds, distribute, tamp, brew, dispense cup, clean tools, return to stand. Milestone demo = tasks b, c, m. Final demo = all of a–v.

**Critical safety rule:** there are three physical carts (Robot 1/2/3), each with slightly different fixture positions. You MUST use the `.rdk` station file and points spreadsheet matching your assigned cart. Mixing them up risks damaging the real robot/tooling.

## Source documents (ask your team for these, or check LEARN/GitLab)

1. **RoboDK intro tutorial** — how to set up RoboDK, jog the robot, run Python scripts via RoboDK's embedded Python interpreter
2. **Assignment brief** — the full task list (a–v), marking scheme, notes on programming constraints
3. **Lab frames PDF** — diagrams of every fixture's local coordinate frame (colored XYZ axes, RGB=XYZ convention), with numbered key-points
4. **Primer** ("Pose and Homogeneous Transforms") — the actual math method. Two worked examples:
   - **Planar case** (Rancilio example): fixture is only rotated about z, second reference point lies flat. Solve rotation via `atan2(cross, dot)`.
   - **Tilted case** (cup stand example): fixture has a known tilt (by inspection, e.g. −90° about y) PLUS an unknown spin. Strip the known tilt from the local reference point first (matrix × vector, not pose × pose), then solve the remaining spin the same way.
5. **GitLab README** — explains the station tree structure, `tools.py`'s `Tools` class (attach/detach/actuate methods — must use these unmodified, never call Tool Programs from Python directly), and the points spreadsheet columns

## Points data (`points.py`)

A dict keyed by spreadsheet key number:
```python
key: (frame, is_origin, fixture_name, x, y, z, description)
```
- `frame` is `"world"` or `"local"`
- `is_origin` is `True`/`False` — whether this point is that fixture's own frame origin
- Every physical point has TWO rows: one in world coords, one in local coords, same key-pair (e.g. Key 35 = Key 10 means "same physical point, recorded in both frames")

## Core method — three cases, pick per fixture

**1. Planar (simple z-rotation only):** works when a second reference point's local z ≈ 0 AND world-to-local segment lengths roughly match. Translation = world origin directly. Rotation via `atan2(cross, dot)` on the x/y displacement.

**2. Known tilt + unknown spin:** when the fixture is tilted at a clean, nameable angle (by inspection — look at the actual lab frames picture). Apply the known tilt matrix to the local reference point (explicit matrix-vector multiply, NOT `pose * transl()`), check the flattened result's leftover axis comes out ≈0, then solve the remaining spin via atan2, combine as `rotz(psi) * known_tilt`.

**3. Two independent measured axes:** when the fixture has no clean nameable tilt (fully arbitrary orientation) but two reference points along genuinely different local axes exist. Normalize each direction to get two columns of the rotation matrix directly (each column = "where that axis points, in world coords" — this is literally Section 5 of the primer, "read a rotation matrix by inspection", just using measured data instead of eyeballing). Compute the third column via cross product to keep the frame right-handed. **Check `x × y == z` exactly** — if you need to flip a sign for physical reasons, flip exactly one other column too, or the frame becomes a mirror reflection (breaks orientation, causes the "half-tilted" bug).

**After the fixture pose:** chain in the placement point's local offset, an axis-mapping (by inspection — how should the tool's own XYZ align with the fixture's XYZ once correctly placed) as a full 3-column rotation matrix, then invert the tool's own local contact-point offset to convert "where the contact point should be" into "where the robot's TCP should be sent."

## File structure per task

```python
def compute_task_X_pose(fixture_fine_tune_mm, other_tunable_args):
    # pure calculation, no robot calls
    # fixture pose -> placement -> axis mapping -> tool offset -> invert
    return pose

def compute_waypoints(target_pose, offsets):
    # generic helper: builds named intermediate poses as local offsets
    # from a target pose. offsets = [(name, (x,y,z)_mm, (rx,ry,rz)_deg), ...]

def run_task_X(UR5, tls, RDK):
    # all edit-in-the-lab tunables declared at the top
    # calls compute_task_X_pose(...), compute_waypoints(...)
    # actual MoveJ/MoveL/tool-actuation sequence

if __name__ == "__main__":
    # sim setup block vs real-robot setup block, clearly separated
    run_task_X(...)
```

Fine-tune values (fixture position correction, approach angle correction) are function **parameters**, set at the top of `run_task_X`, not buried inside the calc function — so they're visible and editable at a glance during a lab session.

## Known gotchas (save yourself the debugging time)

- **RoboDK `ITEM_TYPE_*` constants are unreliable in practice.** Don't trust `RDK.Item(name, TYPE)` filtering blind — if it comes back invalid, loop `RDK.ItemList()` and match by `.Name()` directly, then check `.type` to see what it actually is.
- **`SolveIK_All()` returns a single `Mat`, not a list of solutions.** `len()` on it does NOT tell you how many configurations exist — indexing into it the way you'd index a list gives you one row (one joint's angle), not one full solution. Convert with `.tolist()` and inspect the actual shape before trusting it.
- **Pasting a pose matrix from RoboDK's UI:** wrap it directly in `Mat([[...],[...],[...],[...]])`. Don't try to multiply a `transl()` pose by a rotation to "apply" a matrix to a raw point — for that, write an explicit matrix-vector multiply function.
- **Right-handedness check:** after building any axis-mapping matrix, confirm `cross_product(x_axis, y_axis) == z_axis`. If you flip one column's sign to fix an orientation problem, you must flip exactly one other column too, or you'll get a mirrored (invalid) frame — shows up as an unexplained tilt.
- **`MoveL` guarantees a straight TCP path, NOT that the rest of the arm avoids collisions.** For collision avoidance, back off along the tool's own local z (retreat direction) to build a staging point, and turn on collision detection (Shift-X) while testing.
- **Don't just Cartesian-jog to a solution and hardcode it disconnected from your calculation.** Tutor feedback: compute the target pose properly first, THEN use small adjustments (fine-tune position, approach-angle rotation) to influence which joint configuration `MoveJ` picks — this keeps the "why" documented and defensible in your report, and means corrections propagate automatically if the underlying calc changes.
- **Calculation + calibration is the real industry pattern**, not "cheating." Compute from survey data first, use small measured/tuned corrections for real-world calibration drift — document why each correction was needed.

## Safety

- Real robot testing is lab-timeslot-only, 20 min sessions. Simulate and verify (collision-free, sane joint solutions) before burning a slot.
- Never call Tool Programs from Python — use `tools.py`'s `Tools` class methods only, unmodified.
- Confirm you're using your assigned cart's `.rdk` file and points sheet every session.
