# models.py
#
# Per-robot model configs: servo type, register addresses, joint layout, defaults.

MODELS = {
    "i2rt_yam": {
        "servo_type": "dynamixel",
        "protocol_version": 2.0,
        "baud": 57600,
        "ids": [1, 2, 3, 4, 5, 6, 7],
        "joint_names": {
            1: "shoulder_pan",
            2: "shoulder_lift",
            3: "elbow_flex",
            4: "wrist_flex",
            5: "wrist_roll",
            6: "wrist_yaw",
            7: "gripper",
        },
        "addr_present_position": 132,
        "len_present_position": 4,
        "addr_torque_enable": 64,
        "default_calib": "yam.json",
    },
    "so101": {
        "servo_type": "feetech",
        "protocol_version": 0,
        "baud": 1_000_000,
        "ids": [1, 2, 3, 4, 5, 6],
        "joint_names": {
            1: "shoulder_pan",
            2: "shoulder_lift",
            3: "elbow_flex",
            4: "wrist_flex",
            5: "wrist_roll",
            6: "gripper",
        },
        "addr_present_position": 56,
        "len_present_position": 2,
        "addr_torque_enable": 40,
        "addr_lock": 55,
        "addr_phase": 18,
        "default_calib": "so101.json",
    },
    "so100": {
        "servo_type": "feetech",
        "protocol_version": 0,
        "baud": 1_000_000,
        "ids": [1, 2, 3, 4, 5, 6],
        "joint_names": {
            1: "shoulder_pan",
            2: "shoulder_lift",
            3: "elbow_flex",
            4: "wrist_flex",
            5: "wrist_roll",
            6: "gripper",
        },
        "addr_present_position": 56,
        "len_present_position": 2,
        "addr_torque_enable": 40,
        "addr_lock": 55,
        "addr_phase": 18,
        "default_calib": "so100.json",
    },
}


def get_model(name):
    if name not in MODELS:
        available = ", ".join(sorted(MODELS.keys()))
        raise ValueError(f"Unknown model '{name}'. Available: {available}")
    return MODELS[name]


def get_sdk(servo_type):
    """Import and return the correct servo SDK module based on servo type."""
    if servo_type == "dynamixel":
        import dynamixel_sdk as sdk
        return sdk
    elif servo_type == "feetech":
        import scservo_sdk as sdk
        return sdk
    else:
        raise ValueError(f"Unknown servo type: {servo_type}")
