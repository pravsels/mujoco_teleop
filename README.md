# MuJoCo Teleop

Reads Dynamixel joint positions, normalizes to `[0,1]`, and drives a MuJoCo robot viewer. Default robot is I2RT's YAM. 

## Quickstart

```bash
uv sync
```

Terminal 1 (hardware -> ZMQ):
```bash
uv run python leader_pub.py --port /dev/ttyUSB0 --model i2rt_yam --calib yam.json
```

Terminal 2 (ZMQ -> MuJoCo):
```bash
uv run python mujoco_viewer.py --model i2rt_yam
```

If you do not know your port:
```bash
uv run python find_port.py
```

If you do not have a calibration file yet:
```bash
uv run python calibrate.py --port /dev/ttyUSB0
```
