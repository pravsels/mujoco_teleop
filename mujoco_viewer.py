# mujoco_viewer.py 

import os
os.environ['MUJOCO_GL'] = 'glfw'

import json, argparse, time, zmq
import mujoco, mujoco.viewer
import numpy as np
from utils import make_pub, make_sub

# port for publishing sim arm 
PUB_ADDR = "tcp://*:6001"
# port for subscribing to real arm 
SUB_ADDR = "tcp://localhost:6000"

def run_loop(pub, sub, get_state, apply_state, topic_name, model, data):

    print("\nLaunching viewer...")
    print("Controls:")
    print("  - Mouse: Rotate view")
    print("  - Right mouse: Pan")
    print("  - Scroll: Zoom")
    print("  - Space: Pause/play")
    print("  - ESC: Exit")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        try:
            while viewer.is_running():
                # if follower, apply master's state
                if sub and sub.poll(timeout=0):
                    _, payload = sub.recv_multipart(flags=zmq.NOBLOCK)
                    msg_master = json.loads(payload.decode())
                    apply_state(np.array(msg_master["qpos"], dtype=np.float32))

                # step physics
                mujoco.mj_step(model, data)

                # publish own state
                qpos = get_state()
                msg = {"t": time.time(), "qpos": qpos}
                pub.send_multipart([topic_name.encode(), json.dumps(msg).encode()])

                viewer.sync()

                time.sleep(0.01)  # ~100Hz
        except KeyboardInterrupt:
            pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["master"], help="Set sim as master (default=follower)")
    args = parser.parse_args()
    is_master = args.mode == "master"

    robot_model_name = 'i2rt_yam'
    # Path to model 
    model_path = f"robot_models/{robot_model_name}/scene.xml"
    
    if not os.path.exists(model_path):
        print(f"Model not found: {model_path}")
        print(f"Current directory: {os.getcwd()}")
        print("Make sure you're running from the correct directory!")
        return
    
    print("Loading robot model ...")
    
    # Load model
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    print(f"Model loaded successfully!")
    print(f"  - Joints: {model.njnt}")
    print(f"  - Actuators: {model.nu}")
    print(f"  - Bodies: {model.nbody}")

    # ZeroMQ for publisher and subscriber 
    ctx = zmq.Context()
    pub = make_pub(ctx, PUB_ADDR, f"{robot_model_name}.state_sim")
    sub = None if is_master else make_sub(ctx, SUB_ADDR, f"{robot_model_name}.state_real")

    def get_sim_state():
        return data.qpos[:model.nq].astype(float).tolist()

    def apply_real_state(qpos_real):
        n = min(model.nq, len(qpos_real))
        
        qpos_current = data.qpos[:n]
        delta = qpos_real - qpos_current
        # clip each joint to ±rad
        max_step = 0.1
        delta_clipped = np.clip(delta, -max_step, max_step)
        qpos_next = qpos_current + delta_clipped

        data.qpos[:n] = qpos_next

        mujoco.mj_forward(model, data)
    
    run_loop(pub, sub, get_sim_state, apply_real_state, f"{robot_model_name}.state_sim", model, data)

    pub.close(0)
    if sub:
        sub.close(0)
    ctx.term()

    print("Viewer closed.")

if __name__ == "__main__":
    main()

