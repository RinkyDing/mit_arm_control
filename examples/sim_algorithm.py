"""SDK 示例：默认模拟运动；--observe 只请求查询，可连接实机观察服务。"""
import argparse
import math
import time
from mit_arm_control import ArmClient
from mit_arm_control.config import NAMES

p = argparse.ArgumentParser()
p.add_argument('--socket', default='/tmp/mit-arm-control.sock')
p.add_argument('--duration', type=float, default=5.)
p.add_argument('--observe', action='store_true')
p.add_argument('--set-zero', action='store_true')
p.add_argument('--zero-pose-confirmed', action='store_true')
p.add_argument('--print-rate', type=float, default=1.)
a = p.parse_args()
if not math.isfinite(a.duration) or a.duration < 0 or not 0 < a.print_rate <= 20:
    p.error('duration must be >=0; print-rate must be in (0,20]')
if (a.set_zero or a.zero_pose_confirmed) and not a.observe:
    p.error('--set-zero/--zero-pose-confirmed require --observe')
if a.set_zero and not a.zero_pose_confirmed:
    p.error('place the fixed pose, then pass --zero-pose-confirmed')
with ArmClient(a.socket) as client:
    state = client.get_state()
    if a.observe:
        if not state.get('observation_only'):
            raise RuntimeError('--observe requires a serve --observe-only service')
        try:
            client.observe(set_zero=a.set_zero, workspace_ready=a.zero_pose_confirmed,
                           zero_pose=a.zero_pose_confirmed)
            print('OBSERVING: initialization complete; no enable or MIT targets. You may move the arm by hand.')
            begin = time.monotonic()
            while not a.duration or time.monotonic() - begin < a.duration:
                state = client.get_state()
                if state['state'] == 'FAULT':
                    raise RuntimeError(state['reason'])
                for name, metric in state['metrics'].items():
                    f = state['feedback'].get(name)
                    feedback = 'NO_FEEDBACK' if not f else (
                        f"q={f['q']:+.5f} dq={f['dq']:+.4f} status={f['status']} age={f['age_ms']:.1f}ms"
                    )
                    print(f"{name} TX={metric['tx_hz']:.1f}Hz RX={metric['rx_hz']:.1f}Hz "
                          f"skipped={state['skipped']} backpressure={metric['backpressure']} "
                          f"max_RX_gap={metric['max_rx_gap_ms']:.3f}ms {feedback}")
                time.sleep(1. / a.print_rate)
        except KeyboardInterrupt:
            pass
        finally:
            result = client.stop()
            print('Stop confirmed:', result['stop_confirmed'])
    else:
        if state['backend'] != 'simulation':
            raise RuntimeError('motion example is only for the simulator')
        client.arm(workspace_ready=True, zero_pose=True)  # virtual confirmations only
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
