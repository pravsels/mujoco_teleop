def test_link_collision_data_nonempty():
    import jax.numpy as jnp

    from so101_robot import load_so101_robot

    robot = load_so101_robot()
    data = robot.link_collision_data(jnp.zeros(robot.num_joints))
    assert data.shape[0] >= 1 and data.shape[1] == 4  # (x, y, z, r)
    # Link 4 (fixed jaw) is in oscbf; link 5 (moving jaw) handled separately


def test_with_collision_flag_off_is_empty():
    from so101_robot import load_so101_robot

    robot = load_so101_robot(with_collision=False)
    assert not robot.has_collision_data
