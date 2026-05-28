# SO101 Robust CBF + Workspace Awareness Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make the SO101 safety filter robust against the end-effector driving fast into, and slowly pushing into, the table and the walls of a test rig; let the arm discover its own workspace box autonomously; and package the filter as a portable, dependency-light component that drops into other environments (notably the `remoterobo` edge connector) as the authoritative safety boundary.

**Architecture:** Keep the kinematic velocity-CBF + position-goal pipeline, but split it into a **portable core** (`SafetyFilter`) and **thin adapters** (sim, `sim_to_real`, edge connector). The core does FK + keep-in box barriers (floor + 4 walls + ceiling) over a small gripper sphere cluster + velocity clamp + FK gate + a contact-guard hook. The core imports **no JAX/torch and no transport (no ZMQ)**; transport is the adapter's job. FK is a **pluggable provider** behind a one-method interface: `NumpyKinematics` (exported constants, numpy-only, the Pi default) or `MujocoKinematics` (loads the XML, used in sim/dev and available on the Pi if you prefer shipping the model). A parity test asserts the two FK backends agree to ~1e-4. The heavy **JAX/oscbf stack is offline + sim/dev tooling only**: it derives the sphere model and constants, acts as the reference implementation, and runs the sim harness. A second parity test asserts the numpy core matches the JAX reference, so sim testing validates the exact code that will run on hardware.

**Sim-first (important):** All development and acceptance happens in MuJoCo here — no robot or workstation is required. The hardware path (real Feetech bus, Pi edge connector) is exercised through fakes/simulated buses, and real bring-up is a short checklist deferred to when hardware is available. Every adapter seam is designed so swapping the simulated bus for the real one is a one-line change.

**Tech Stack:**
- Portable core runtime: Python + numpy. No JAX/torch, no transport. Optional `mujoco` only if `MujocoKinematics` is chosen.
- Offline/sim/dev: MuJoCo, JAX, cbfpy, oscbf (sibling repo `../oscbf`), ZMQ (sim teleop transport only).
- Hardware adapters (stubbed with fakes until hardware is available): lerobot/Feetech bus, `remoterobo` edge connector.

**Where it runs (decided):** The authoritative filter runs **on the Pi** inside the edge connector's `Goal_Position` path (numpy core, local low-level reads, true boundary). The workstation may optionally run the richer JAX filter as a first pass (defense in depth). The existing ±30° interlock becomes one selectable policy via a pluggable interface.

**Out of scope (YAGNI):** whole-body sphere model, self-collision avoidance, torque-OSCBF on hardware, moving-obstacle barriers, JAX on the Pi.

**Known gap (future work):** The moving jaw sphere is computed at the closed-jaw position. When the gripper opens, the moving jaw tip extends further (e.g. downward toward the table). The filter does NOT track this — it could miss a collision if the jaw opens near a boundary. Fix: add the gripper as a 6th joint in `NumpyKinematics`, pass 6 joint values to the filter.

---

## How verification works in this plan

Every task ends with a **Visual Check**: an exact command you run, and an explicit **PASS/FAIL you can see in the MuJoCo viewer** without reading code. Each task also has a small automated test for regression, but the human sign-off gate is visual. Do not move to the next task until the visual check passes.

**Cross-repo note:** code lives in three repos — `mujoco_teleop` (this repo, incl. the new `so101_safety/` portable package), `../oscbf` (the paper code, dev/offline only), and `../alpha-robotics/remoterobo/edge` (the Pi connector, Task 8). Confirm `oscbf` is installed editable (`pip show oscbf`). Tests for `mujoco_teleop` go under `tests/`; portable-core tests under `so101_safety/tests/`; SO101 config/geometry tests under `../oscbf/test/`; edge tests under the edge repo's `tests/`.

**Commit cadence:** commit after each task's visual check passes.

**Environment & sandbox constraints (learned during Task 0 — IMPORTANT):**
- Use the project venv: `.venv/bin/python` (uv-managed; run tests via `.venv/bin/python -m pytest`). `oscbf` is editable-installed into `.venv` via its deps plus a `.pth` at `.venv/.../site-packages/oscbf_editable.pth` pointing at `../oscbf` and `../oscbf/test`, so `import oscbf` and `import test_so101_real` work.
- **numpy is pinned to 2.4.6.** The 1.26.4 wheel segfaults in LAPACK on this machine; do not let a resolver downgrade it.
- **`../oscbf` is read-only** (sandbox blocks writes there). Therefore `oscbf` stays pristine: we do NOT edit it. All SO101-specific code (sphere model, robot builder, CBF config) lives in `mujoco_teleop`. We still *import* oscbf classes (`Manipulator`, `OSCBFVelocityConfig`, etc.).
- `lerobot` is not installed in this sim env; the Feetech current/load register confirmation is deferred to Task 5.

**Portability principle (read before coding):** The runtime core must stay free of JAX/torch and of any transport (no ZMQ), so the same code runs in sim, in `sim_to_real.py`, and on the Pi edge connector. FK is injected via a provider, so `mujoco` is an *optional* backend, never a hard core dependency. Anything needing JAX/oscbf (sphere derivation, constant export, the reference filter, the sim harness) is **offline or dev tooling** and must not be imported by the core. The portable core lives in a standalone package directory (`so101_safety/`) with a minimal `pyproject.toml` (deps: `numpy`, `scipy`; extra: `mujoco`) so it can be `pip install`ed into the edge repo. `scipy` is included because the CBF-QP uses SLSQP — it is numpy-based, Pi-friendly, and has no JAX/torch dependency. Adapters own transport and the bus; the core only sees arrays in, arrays out.

---

## Task 0: Verify environment and assumptions

**Files:**
- Test: `tests/test_env_smoke.py`

**Step 1: Confirm the current pipeline runs and the sphere overlay is empty today**

Run:
```bash
python mujoco_viewer.py --model so101 --show-oscbf-spheres --table-z 0.05
```
Expected (documents the starting point): arm renders, table plane shows, **no collision spheres drawn** (confirms `link_collision_data` is empty for SO101 today).

**Step 2: Confirm Feetech current/load is readable (drives the contact-guard design)**

Write `tests/test_env_smoke.py` with a test that imports `oscbf`, `cbfpy`, `jax`, and asserts `load_robot()` returns a 5-joint manipulator:
```python
def test_oscbf_so101_loads():
    from test_so101_real import load_robot
    robot = load_robot()
    assert robot.num_joints == 5
```
Then manually verify which Feetech register exposes current/load (candidates: `Present_Current`, `Present_Load`) by reading the lerobot Feetech bus register table. Record the exact register name in this plan file under Task 5 before implementing it.

**Step 3: Run**
```bash
pytest tests/test_env_smoke.py -v
```
Expected: PASS.

**Visual Check:** Step 1 shows the arm with **no spheres** (baseline). This is the "before" picture.

**Step 4: Commit**
```bash
git add tests/test_env_smoke.py docs/plans/2026-05-28-so101-robust-cbf-workspace.md
git commit -m "chore: smoke test + document SO101 CBF baseline"
```

---

## Task 1: Derive a gripper/wrist sphere cluster from the MuJoCo model

**Files:**
- Create: `tools/derive_so101_spheres.py`
- Create: `so101_collision_model.py` (generated output, checked in, at mujoco_teleop repo root — NOT in oscbf)
- Test: `tests/test_derive_so101_spheres.py`

**Approach:** Enumerate `class="collision"` geoms on the `wrist`, `gripper`, and `moving_jaw_so101_v1` bodies. For each, use MuJoCo's `model.geom_rbound` as the sphere radius and the geom pose expressed in its parent link frame as the center. Map MuJoCo bodies to oscbf link indices (the oscbf chain joints are `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll`). Emit `so101_collision_data = {"positions": (...per link...), "radii": (...)}` in the franka model's format (see `../oscbf/oscbf/core/franka_collision_model.py`).

**Step 1: Write the failing test**
```python
def test_so101_spheres_are_reasonable():
    from so101_collision_model import so101_collision_data
    positions = so101_collision_data["positions"]
    radii = so101_collision_data["radii"]
    flat_r = [r for link in radii for r in link]
    assert 3 <= len(flat_r) <= 6
    assert all(0.005 < r < 0.08 for r in flat_r)   # gripper-scale spheres
```

**Step 2: Run to verify it fails**
```bash
pytest tests/test_derive_so101_spheres.py -v
```
Expected: FAIL (module does not exist yet).

**Step 3: Implement `tools/derive_so101_spheres.py`**

Load `robot_models/so101/scene.xml`, iterate `model.ngeom`, select geoms whose body is in `{wrist, gripper, moving_jaw_so101_v1}` and `geom_contype/conaffinity` indicate the collision class, compute center in link frame from `geom_pos`/`geom_quat` relative to the mapped joint frame, take `model.geom_rbound[i]` as radius, and write the data file. Keep only the closest-to-tip 3-5 spheres if more are found.

**Step 4: Run to verify it passes**
```bash
python tools/derive_so101_spheres.py
pytest tests/test_derive_so101_spheres.py -v
```
Expected: PASS.

**Visual Check:** deferred to Task 2 (overlay needs `load_robot` wired). 

**Step 5: Commit**
```bash
git add tools/derive_so101_spheres.py so101_collision_model.py tests/test_derive_so101_spheres.py
git commit -m "feat: derive SO101 gripper sphere cluster from MuJoCo meshes"
```

---

## Task 2: Local SO101 robot builder that wires in the sphere model

oscbf stays pristine. Instead of editing oscbf's `load_robot`, add a local builder `load_so101_robot()` here that reuses oscbf's `Manipulator.from_urdf` + the SO101 URDF constant but passes `collision_data`. Migrate `mujoco_viewer.py` / `sim_to_real.py` to import from this builder.

**Files:**
- Create: `so101_robot.py` (repo root) — `load_so101_robot(with_collision=True)`
- Modify: `mujoco_viewer.py`, `sim_to_real.py` (swap `from test_so101_real import load_robot` → local builder)
- Test: `tests/test_so101_safety.py`

**Step 1: Write the failing test**
```python
def test_link_collision_data_nonempty():
    import jax.numpy as jnp
    from so101_robot import load_so101_robot
    robot = load_so101_robot()
    data = robot.link_collision_data(jnp.zeros(robot.num_joints))
    assert data.shape[0] >= 3 and data.shape[1] == 4  # (x,y,z,r)
```

**Step 2: Run to verify it fails** — Expected: FAIL (module missing).

**Step 3: Implement** `so101_robot.py`:
```python
from test_so101_real import SO101_URDF  # reuse oscbf's URDF path constant
from oscbf.core.manipulator import Manipulator
from so101_collision_model import so101_collision_data

def load_so101_robot(with_collision=True):
    cd = so101_collision_data if with_collision else None
    return Manipulator.from_urdf(SO101_URDF, collision_data=cd)
```
(Confirm the exact URDF constant name / `from_urdf` signature in `../oscbf/test/test_so101_real.py` while implementing.)

**Step 4: Run to verify it passes** — Expected: PASS.

**Visual Check:**
```bash
python mujoco_viewer.py --model so101 --show-oscbf-spheres --table-z 0.05
```
**PASS:** 3-5 blue spheres now hug the gripper/wrist and move with the arm. **FAIL:** no spheres, or spheres float away from the gripper.

**Step 5: Commit**
```bash
git add so101_robot.py mujoco_viewer.py sim_to_real.py tests/test_so101_safety.py
git commit -m "feat: local SO101 builder with gripper collision spheres"
```

---

## Task 3: Workspace containment config (box: floor + walls + ceiling)

**Files:**
- Create: `so101_workspace.py` (repo root) (`WorkspaceContainmentConfig`)
- Test: `tests/test_so101_safety.py`

**Approach:** Velocity-CBF (`OSCBFVelocityConfig`). `h_1` returns, for every gripper sphere center `p_i` (radius `r_i`) and box `[lo, hi]` on x/y/z:
`concat(p_i - lo - r_i, hi - p_i - r_i)`. A `buffer` shrinks the box (`lo += buffer`, `hi -= buffer`) to absorb one-step discrete overshoot. Reuse the 5-DOF `P`/`q` override from `TableAvoidanceConfig`.

**Step 1: Write the failing test**
```python
def test_box_margin_sign():
    import jax.numpy as jnp
    from so101_robot import load_so101_robot
    from so101_workspace import WorkspaceContainmentConfig
    robot = load_so101_robot()
    cfg = WorkspaceContainmentConfig(robot, lo=(-0.3,-0.3,0.05), hi=(0.3,0.3,0.6))
    h = cfg.h_1(jnp.zeros(robot.num_joints))
    assert bool((h > -1.0).all())  # finite, well-formed
```

**Step 2: Run to verify it fails** — Expected: FAIL (module missing).

**Step 3: Implement `WorkspaceContainmentConfig`** (mirror `so101_table_avoidance.py`, extend `h_1` to the full box over all spheres, add `buffer`).

**Step 4: Run to verify it passes** — Expected: PASS.

**Visual Check:** deferred to Task 4 (needs relay integration + box drawing).

**Step 5: Commit**
```bash
git add so101_workspace.py tests/test_so101_safety.py
git commit -m "feat: SO101 workspace box containment CBF"
```

---

## Task 3.5: Portable numpy-only `SafetyFilter` core (+ JAX parity)

**This is the keystone task** — the reusable component that ships to every environment, including the Pi. Tasks 4-8 become adapters around it.

**Files:**
- Create: `so101_safety/so101_safety/__init__.py`
- Create: `so101_safety/so101_safety/kinematics.py` (`Kinematics` protocol + `NumpyKinematics` from exported constants)
- Create: `so101_safety/so101_safety/kinematics_mujoco.py` (`MujocoKinematics`, optional backend; imported lazily)
- Create: `so101_safety/so101_safety/filter.py` (`SafetyFilter`, takes a `Kinematics` provider)
- Create: `so101_safety/so101_safety/constants.py` (generated kinematic + sphere constants)
- Create: `so101_safety/pyproject.toml` (`dependencies = ["numpy", "scipy"]`, `[project.optional-dependencies] mujoco = ["mujoco"]`)
- Create: `tools/export_so101_constants.py` (offline: dumps oscbf kinematics + spheres into `constants.py`)
- Test: `so101_safety/tests/test_fk_parity.py`, `so101_safety/tests/test_filter.py`

**Interface (stable, environment-agnostic):**
```python
class Kinematics(Protocol):
    def sphere_positions(self, q) -> np.ndarray: ...   # (n_spheres, 3) world frame
    def ee_position(self, q) -> np.ndarray: ...         # (3,)

class SafetyFilter:
    def __init__(self, kinematics: Kinematics, box_lo, box_hi, *,
                 buffer=0.01, max_vel=None, alpha=5.0, dt=0.05): ...
    def filter(self, current_q, desired_q, feedback=None) -> np.ndarray:
        """current_q, desired_q in radians. feedback may carry motor currents
        for the contact guard (Task 5). Returns a safe joint target in radians."""
```
Internally: provider FK -> sphere world positions -> box-distance barriers -> CBF-QP velocity clamp -> velocity cap -> FK gate. The core never imports `mujoco`, `jax`, or any transport; `NumpyKinematics` is the default, `MujocoKinematics` is opt-in.

**Jacobian strategy:** `tools/export_so101_constants.py` computes the analytical per-sphere Jacobian `∂sphere_pos/∂q` from the DH chain (using the oscbf `Manipulator`) and emits it as a numpy function in `constants.py`. This keeps the runtime Jacobian evaluation pure numpy with no finite-diff overhead and no JAX at runtime.

**QP solver:** The CBF-QP is small (5 decision variables, ≤30 linear constraints: up to 5 spheres × 6 box walls). Use `scipy.optimize.minimize` (SLSQP) — scipy is numpy-based, Pi-friendly, and has no JAX/torch dependency. `scipy` is added to `pyproject.toml` as a core dependency alongside `numpy`. The QP formulation is: minimise `||δq - δq_nom||²` subject to `J_sphere_i^T · n_wall · δq ≥ -α · h_i` for all active sphere-wall pairs.

**Step 1: Write the failing FK parity tests** (two: numpy-vs-oscbf, and numpy-vs-mujoco)
```python
def test_numpy_fk_matches_oscbf():
    import numpy as np, jax.numpy as jnp
    from so101_safety.kinematics import NumpyKinematics
    from so101_robot import load_so101_robot          # JAX reference (dev only)
    robot = load_so101_robot()
    k = NumpyKinematics()
    for _ in range(20):
        q = np.random.uniform(-1.0, 1.0, 5)
        ref = np.asarray(robot.ee_position(jnp.asarray(q)))
        np.testing.assert_allclose(k.ee_position(q), ref, atol=1e-4)

def test_numpy_fk_matches_mujoco():
    import numpy as np
    from so101_safety.kinematics import NumpyKinematics
    from so101_safety.kinematics_mujoco import MujocoKinematics
    n, m = NumpyKinematics(), MujocoKinematics("robot_models/so101/scene.xml")
    for _ in range(20):
        q = np.random.uniform(-1.0, 1.0, 5)
        np.testing.assert_allclose(n.sphere_positions(q), m.sphere_positions(q), atol=1e-4)
```

**Step 2: Run to verify it fails**
```bash
pytest so101_safety/tests/test_fk_parity.py -v
```
Expected: FAIL (no module / constants yet).

**Step 3: Implement** `tools/export_so101_constants.py` (read oscbf `Manipulator` joint axes, parent transforms, ee offset, sphere data, and per-sphere Jacobian functions; write `constants.py` with all of these as numpy arrays/callables), then numpy `kinematics.py` (FK + sphere Jacobian) and `filter.py` (CBF-QP via scipy SLSQP). Also update `pyproject.toml` to add `scipy` alongside `numpy` as a core dependency.

**Step 4: Run to verify it passes**
```bash
python tools/export_so101_constants.py
pytest so101_safety/tests/test_fk_parity.py so101_safety/tests/test_filter.py -v
```
Expected: PASS (numpy FK within 1e-4 of JAX; filter clamps a wall-crossing target).

**Step 5: Filter parity vs the JAX reference**
Add a test asserting `SafetyFilter.filter` agrees with the oscbf `WorkspaceContainmentConfig` safety_filter to a tolerance on random states near the box faces. This is what lets "tested in sim" certify the Pi runtime.

**Visual Check:** deferred to Task 4 (the relay will now be driven by this core).

**Step 6: Commit**
```bash
git add so101_safety tools/export_so101_constants.py
git commit -m "feat: portable numpy SafetyFilter core with JAX FK/filter parity"
```

---

## Task 4: Drive the relay and viewer from the portable core

**Files:**
- Modify: `cbf_relay.py` and `sim_to_real.py` (replace inline CBF call with `so101_safety.SafetyFilter`; add `--box-min/--box-max`, `--buffer`, `--max-vel-deg`)
- Modify: `mujoco_viewer.py` / `sim_to_real.py` `draw_*` (draw the box walls, not just the plane)
- Test: `tests/test_box_clamp.py`

**Step 1: Write the failing test** — synthetic: a nominal velocity driving a sphere past a wall is clamped so the predicted next position stays inside `box - buffer`.

**Step 2: Run to verify it fails** — Expected: FAIL.

**Step 3: Implement** — import `SafetyFilter` from the portable core (NOT the JAX config; the relay must not import oscbf at runtime), add CLI args, add `draw_box()` (6 translucent faces; turn red on contact), clamp nominal velocity to `max_vel`. This proves the portable core drives real behavior, not just unit tests.

**Step 4: Run to verify it passes** — Expected: PASS.

**Visual Check (the core case-1 demo):**
```bash
# terminal 1
python mujoco_viewer.py --model so101 --show-oscbf-spheres --table-z 0.08
# terminal 2 (drive a fast straight-down + sideways target via leader/sliders)
python sim_to_real.py --port SIM --min-safe-ee-z 0.08 \
  --box-min " -0.25 -0.25 0.08" --box-max "0.25 0.25 0.55" --max-vel-deg 60 --buffer 0.01
```
**PASS:** when you slam the target down or sideways, the gripper spheres **decelerate and stop at the box faces** with a small visible gap; faces flash red at the boundary but spheres never cross. **FAIL:** spheres cross any face, oscillate, or stop far away from the face (over-conservative).

**Step 5: Commit**
```bash
git add cbf_relay.py sim_to_real.py mujoco_viewer.py tests/test_box_clamp.py
git commit -m "feat: workspace box filtering + box visualization in relay/viewer"
```

---

## Task 5: Contact guard (slow-push / high-torque case)

**Files:**
- Create: `so101_safety/so101_safety/contact_guard.py` (numpy-only `ContactGuard`, part of the portable core)
- Modify: `so101_safety/so101_safety/filter.py` (accept a guard via the `feedback` arg)
- Modify: `cbf_relay.py` / `sim_to_real.py` to pass feedback into the filter
- Test: `so101_safety/tests/test_contact_guard.py`

**Register to use (fill in from Task 0):** `__________` (e.g. `Present_Current` / `Present_Load`). In sim, use MuJoCo `data.qfrc_constraint` / contact force as the analog.

**Why it lives in the core and runs Pi-side:** the contact guard needs live motor current, which is only available where the bus is (the Pi). Keeping it in the numpy core means the same guard runs in sim and on the Pi; the adapter just supplies the current readings.

**Step 1: Write the failing test** — feed a synthetic per-joint signal series; assert the guard flags contact after `N` consecutive samples above `baseline + threshold`, and clears after release.

**Step 2: Run to verify it fails** — Expected: FAIL.

**Step 3: Implement** — `ContactGuard` with `calibrate(samples)`, `update(currents) -> ContactState`, and a response that zeroes the commanded velocity component pushing further in (hold) + sets a flag. Wire a `beep()`/on-screen red indicator on trigger.

**Step 4: Run to verify it passes** — Expected: PASS.

**Visual Check (the core case-2 demo, sim):**
```bash
python sim_to_real.py --port SIM --min-safe-ee-z 0.08 --box-min " -0.25 -0.25 0.08" \
  --box-max "0.25 0.25 0.55" --enable-contact-guard
```
Then command a **slow** sustained push into the table/floor.
**PASS:** as contact force builds, the arm **stops pushing and holds/backs off slightly**, and the on-screen indicator/beep fires; commanded goal stops advancing into the surface. **FAIL:** the arm keeps grinding into the surface, or the guard fires constantly in free space.

**Step 5: Commit**
```bash
git add so101_safety/so101_safety/contact_guard.py so101_safety/so101_safety/filter.py so101_safety/tests/test_contact_guard.py cbf_relay.py sim_to_real.py
git commit -m "feat: current/contact-force guard for slow-push protection"
```

---

## Task 6: Autonomous workspace discovery ("the arm finds its rig")

**Files:**
- Create: `explore_workspace.py`
- Create: workspace calibration output `calib/so101_workspace.json`
- Test: `tests/test_workspace_fit.py`

**Approach:** Under conservative barriers (joint limits + a generous initial box + the contact guard), jog the EE on a coarse grid / random walk. Record EE positions that are reachable without contact; when the contact guard fires, log that EE position as a **boundary sample** (a rig wall/floor). Fit an axis-aligned keep-in box = per-axis `[max(min_reachable)+buffer ... ]` bounded inside the boundary samples. Save to JSON; the relay loads it as `--box-min/--box-max`.

**Step 1: Write the failing test** — given synthetic reachable + boundary samples, `fit_workspace_box()` returns a box strictly inside the boundary samples and containing the reachable cloud.

**Step 2: Run to verify it fails** — Expected: FAIL.

**Step 3: Implement** the explorer loop + `fit_workspace_box()` + JSON save/load.

**Step 4: Run to verify it passes** — Expected: PASS.

**Visual Check (sim, with rig walls present):**
```bash
python explore_workspace.py --model so101 --rig narrow_box --out calib/so101_workspace.json
# then re-run the viewer with the discovered box
python sim_to_real.py --port SIM --workspace-file calib/so101_workspace.json
```
**PASS:** you watch the arm sweep around, gently touch each wall/floor, and the **resulting drawn box matches the rig walls** (snug, just inside them). **FAIL:** box is larger than the rig (would allow collisions) or collapses to near-nothing.

**Step 5: Commit**
```bash
git add explore_workspace.py tests/test_workspace_fit.py calib/so101_workspace.json
git commit -m "feat: autonomous SO101 workspace discovery via contact"
```

---

## Task 7: Parametrized sim test-rig robustness harness

**Files:**
- Create: `robot_models/so101/rigs/` (MuJoCo include snippets: `table_only.xml`, `narrow_box.xml`, `tall_box.xml`, `tilted_wall.xml`)
- Create: `tests/test_rig_robustness.py` (automated) + `run_rig_robustness.py` (visual runner)
- Test: `tests/test_rig_robustness.py`

**Approach:** Drive the **portable `SafetyFilter`** (the exact Pi runtime), not the JAX reference, so passing here certifies the hardware path. For each rig config, build/load the box, then run adversarial trajectories: fast straight-in at each face + a slow sustained push. Record per config: `min h` (must stay `> 0`), penetration depth (must be `0`), tracking error away from the boundary, and filter rate (Hz). Assert no penetration and that mean off-boundary tracking error stays below a threshold (performance, not just safety).

**Step 1: Write the failing test**
```python
@pytest.mark.parametrize("rig", ["table_only", "narrow_box", "tall_box", "tilted_wall"])
def test_no_penetration_and_tracks(rig):
    result = run_rig(rig)                # headless
    assert result.min_h > 0.0
    assert result.max_penetration == 0.0
    assert result.mean_offboundary_error < 0.01   # 1 cm
    assert result.solve_hz > 100
```

**Step 2: Run to verify it fails** — Expected: FAIL (runner missing).

**Step 3: Implement** rig snippets, headless `run_rig()`, and the visual `run_rig_robustness.py`.

**Step 4: Run to verify it passes**
```bash
pytest tests/test_rig_robustness.py -v
```
Expected: PASS for all four rigs.

**Visual Check (final acceptance):**
```bash
python run_rig_robustness.py --rig narrow_box --view
python run_rig_robustness.py --rig tilted_wall --view
```
**PASS:** for each rig, you watch adversarial fast and slow pushes at every face; **nothing ever turns red / penetrates**, the arm tracks crisply when away from walls, and the printed summary table shows `PASS` for every config. **FAIL:** any penetration (red), sluggish/over-conservative motion away from walls, or a `FAIL` row.

**Step 5: Commit**
```bash
git add robot_models/so101/rigs tests/test_rig_robustness.py run_rig_robustness.py
git commit -m "test: parametrized sim test-rig robustness harness"
```

---

## Task 8: `remoterobo` edge connector adapter (the portability payoff)

**Files:**
- Create: `../alpha-robotics/remoterobo/edge/src/remoterobo_edge/safety.py` (pluggable `SafetyPolicy` protocol + two policies)
- Modify: `../alpha-robotics/remoterobo/edge/src/remoterobo_edge/connector.py` (route `_check_safe_action` through the selected policy)
- Modify: `../alpha-robotics/remoterobo/edge/pyproject.toml` and `requirements-pi.txt` (add `so101-safety` + `numpy`; NO jax)
- Test: `../alpha-robotics/remoterobo/edge/tests/test_safety_policies.py`

**Approach:** Define a `SafetyPolicy` protocol with `filter_goal(current_deg, proposed_deg) -> safe_deg`. Provide:
- `DeltaLimitPolicy` — the existing ±30° behavior, refactored behind the interface (default; zero new deps).
- `WorkspaceCbfPolicy` — wraps `so101_safety.SafetyFilter` + `ContactGuard`, converting Feetech degrees <-> radians, using the connector's local `sync_read("Present_Position")` for `current_q` and `Present_Current` for the guard. Loads the discovered workspace box from `calib/so101_workspace.json`.
Selection is via env var (e.g. `REMOTEROBO_SAFETY_POLICY=workspace_cbf`), satisfying the "configurable/pluggable" decision. The CBF policy filters every `Goal_Position` regardless of which job/policy the workstation runs — the true safety boundary.

**Step 1: Write the failing test**
```python
def test_workspace_policy_clamps_goal_into_box():
    from remoterobo_edge.safety import WorkspaceCbfPolicy
    policy = WorkspaceCbfPolicy(box_lo=(-0.25,-0.25,0.08), box_hi=(0.25,0.25,0.55))
    current_deg = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0,
                   "wrist_flex": 0.0, "wrist_roll": 0.0}
    unsafe = dict(current_deg, shoulder_lift=60.0)  # drives EE down into the floor
    safe = policy.filter_goal(current_deg, unsafe)
    assert safe["shoulder_lift"] < 60.0            # clamped back

def test_delta_policy_matches_legacy_30deg():
    from remoterobo_edge.safety import DeltaLimitPolicy
    policy = DeltaLimitPolicy(max_delta_deg=30.0)
    cur = {"shoulder_pan": 0.0}
    assert policy.filter_goal(cur, {"shoulder_pan": 90.0}) is None  # rejected/tripped
```

**Step 2: Run to verify it fails**
```bash
cd ../alpha-robotics/remoterobo/edge && pytest tests/test_safety_policies.py -v
```
Expected: FAIL (module missing).

**Step 3: Implement** `safety.py` and route `_check_safe_action` through `self._policy` (constructed from env). Keep the trip-and-return-to-rest behavior for hard rejections; the CBF policy instead *clamps* and forwards the safe goal.

**Step 4: Run to verify it passes**
```bash
cd ../alpha-robotics/remoterobo/edge && pytest tests/test_safety_policies.py -v
pip install -e ../../../mujoco_teleop/so101_safety   # numpy-only, Pi-friendly
```
Expected: PASS; install pulls in only numpy.

**Visual / integration check (no Pi hardware needed):**
Run the edge connector locally against a simulated bus (or the existing `test_edge_package.py` harness), send a `Goal_Position` that would drive the EE through the floor, and confirm the connector forwards a **clamped** goal instead of the raw one. **PASS:** logged outgoing goal stays inside the box; legacy mode still trips at ±30°. **FAIL:** raw unsafe goal is forwarded, or the import drags in JAX.

**Step 5: Commit** (in the edge repo)
```bash
cd ../alpha-robotics/remoterobo/edge
git add src/remoterobo_edge/safety.py src/remoterobo_edge/connector.py pyproject.toml requirements-pi.txt tests/test_safety_policies.py
git commit -m "feat: pluggable CBF/workspace safety policy in edge connector"
```

---

## Final acceptance summary (what "done & robust" looks like)

Run, in sim, with at least two rig configs:
1. **Fast drive-in** at floor and each wall → spheres brake and stop inside the box (case 1). 
2. **Slow sustained push** into a surface → contact guard holds/backs off (case 2). 
3. **Autonomous exploration** produces a box that visually matches the rig walls. 
4. **Robustness harness** prints all-`PASS` (driving the portable core) with zero penetration and tracking error under threshold away from boundaries.
5. **Portability:** the edge connector, with `REMOTEROBO_SAFETY_POLICY=workspace_cbf`, clamps an unsafe streamed `Goal_Position` using the same numpy core, installing only numpy on the Pi (no JAX). Legacy ±30° mode still available.

Self-collision (case 3) is intentionally excluded per scoping.

All of the above runs in MuJoCo with no robot or workstation present.

---

## Hardware bring-up checklist (deferred until hardware is available)

These steps are NOT part of sim development; they are the short, well-isolated swaps to go live. Each replaces a fake with the real thing behind an already-tested seam:
1. Confirm the real Feetech current register name matches Task 0's finding; calibrate per-joint free-motion current baselines on the real arm (the `ContactGuard.calibrate` entry point already exists).
2. In `sim_to_real.py`, point the bus adapter at the real `SO101Follower` (already implemented) instead of the sim `SIM` bus.
3. On the Pi, `pip install so101_safety` (numpy-only) and set `REMOTEROBO_SAFETY_POLICY=workspace_cbf`; copy `calib/so101_workspace.json`. Choose `NumpyKinematics` (default) or ship the XML for `MujocoKinematics`.
4. Run the autonomous workspace discovery on the real arm in the real rig to capture its true box.
5. Re-run the adversarial pushes at low speed on hardware as a final acceptance.

---

## Execution Handoff

Two execution options:

1. **Subagent-Driven (this session)** — I dispatch a fresh subagent per task and review between tasks. Faster iteration, you sign off each visual check.
2. **Parallel Session (separate)** — open a new session using superpowers:executing-plans for batch execution with checkpoints.

Which approach? And should I set up an isolated git worktree first (recommended, since this touches `mujoco_teleop`, `../oscbf`, and the `remoterobo/edge` repo)?
