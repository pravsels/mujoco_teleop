def test_so101_spheres_are_reasonable():
    from so101_collision_model import so101_collision_data

    positions = so101_collision_data["positions"]
    radii = so101_collision_data["radii"]

    assert len(positions) == 5 and len(radii) == 5

    flat_r = [r for link in radii for r in link]
    assert 1 <= len(flat_r) <= 6
    assert all(0.003 < r < 0.08 for r in flat_r)  # jaw-tip spheres

    # spheres must hang off the last two links (wrist_flex, wrist_roll)
    assert len(radii[3]) + len(radii[4]) == len(flat_r)
