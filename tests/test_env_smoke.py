"""Environment smoke test + documented baseline for the SO101 CBF work.

Baseline at start of this effort:
- `oscbf` is editable-installed into this repo's `.venv` (deps + a `.pth`
  pointing at ../oscbf and ../oscbf/test). numpy is pinned to 2.4.6; the
  1.26.4 wheel segfaults in LAPACK on this machine.
- `link_collision_data` is empty for the SO101 today, so the viewer's
  `--show-oscbf-spheres` draws nothing and the CBF only constrains a single
  end-effector point. Tasks 1-2 add a gripper sphere cluster.
- Contact guard (Task 5) needs a motor current/load reading. `lerobot`
  (which defines the SO101 Feetech control table) is NOT installed in this
  sim environment; the STS3215 registers are `Present_Current` /
  `Present_Load`. Confirm the exact lerobot key in Task 5.
"""


def test_oscbf_so101_loads():
    from test_so101_real import load_robot

    robot = load_robot()
    assert robot.num_joints == 5
