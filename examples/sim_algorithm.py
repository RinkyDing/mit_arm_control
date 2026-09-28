"""Simulation-only algorithm example; refuses a hardware service."""
import argparse
import math
import time
from mit_arm_control import ArmClient
from mit_arm_control.config import NAMES

p = argparse.ArgumentParser()
p.add_argument('--socket', default='/tmp/mit-arm-control.sock')
p.add_argument('--duration', type=float, default=5.)
a = p.parse_args()
with ArmClient(a.socket) as client:
    if client.get_state()['backend'] != 'simulation':
        raise RuntimeError('this example is only for the simulator')
    client.arm(supported=True, zero_pose=True)  # virtual confirmations only
    initial = {n: dict(q_des=0., dq_des=0., kp=2., kd=.1, tau_ff=0.) for n in NAMES}
    client.submit(initial)
    begin = time.monotonic()
    try:
        while time.monotonic()-begin < a.duration:
            t = time.monotonic()-begin
            # Smooth, small position target. This is not a real-arm trajectory.
            q = .02*(1-math.cos(2*math.pi*.25*t))
            dq = .02*2*math.pi*.25*math.sin(2*math.pi*.25*t)
            client.submit({n: dict(q_des=q, dq_des=dq, kp=2., kd=.1, tau_ff=0.) for n in NAMES})
            time.sleep(.005)  # algorithm ~200 Hz, transport independently ~1 kHz
    finally:
        state = client.stop()
        print('Stop confirmed:', state['stop_confirmed'])
        print('Final state:', state['state'])
