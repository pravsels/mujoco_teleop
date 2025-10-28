# utils.py 

import zmq 

def make_pub(ctx, addr, topic_name):
    pub = ctx.socket(zmq.PUB)
    pub.bind(addr)
    print(f"Publishing on {addr}, topic = {topic_name}")
    return pub

def make_sub(ctx, addr, topic_name):
    sub = ctx.socket(zmq.SUB)
    sub.connect(addr)
    sub.setsockopt(zmq.SUBSCRIBE, topic_name.encode())
    sub.setsockopt(zmq.CONFLATE, 1)
    print(f"Subscribed to {addr}, topic = {topic_name}")
    return sub
