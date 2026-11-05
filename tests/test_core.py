import copy
import json
from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch
from mit_arm_control.backend import SimBackend
from mit_arm_control.config import load_config, readiness, NAMES
from mit_arm_control.controller import Controller
from mit_arm_control.ipc import AsyncLogger
from mit_arm_control.protocol import *
from mit_arm_control.safety import validate_command, SafetyError

ROOT = Path(__file__).resolve().parents[1]


class Clock:
    def __init__(self): self.now = 10.
    def __call__(self): return self.now


class VirtualBus(SimBackend):
    def poll(self, timeout=0):
        self.clock.now += max(timeout, .000001)
        return super().poll(0)


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.c = load_config(ROOT/'configs/simulation.json')
        self.clock = Clock()
        self.bus = VirtualBus(self.c['joints'], self.clock)
        self.ctl = Controller(self.c, self.bus, clock=self.clock)
        self.seq = 0

    def cmd(self):
        self.seq += 1
        return dict(seq=self.seq, timestamp=self.clock(), joints={n: dict(q_des=0.,dq_des=0.,kp=2.,kd=.1,tau_ff=0.) for n in NAMES})

    def start(self):
        self.ctl.arm()
        self.assertEqual(self.ctl.state, 'READY')
        self.assertTrue(all(s['status'] == 0 for s in self.bus.state.values()))
        self.ctl.submit(self.cmd())
        self.assertEqual(self.ctl.state, 'RUNNING')

    def cycle(self, n=1):
        for _ in range(n):
            self.clock.now += .001
            if self.ctl.state in self.ctl.ACTIVE:
                self.ctl.submit(self.cmd())
            self.ctl.step()

    def test_constructor_and_import_have_no_motor_writes(self):
        self.assertEqual(self.bus.sent, 0)
        self.assertEqual(self.ctl.state, 'IDLE')

    def test_light_snapshot_exposes_state_without_scanning_statistics(self):
        self.start()
        metric = self.ctl.metrics['J1']
        with patch.object(metric, 'snapshot', side_effect=AssertionError('heavy statistics')):
            state = self.ctl.snapshot(statistics=False)
        self.assertEqual(state['last_seq'], self.seq)
        self.assertEqual(state['state'], 'RUNNING')
        self.assertEqual(state['protocol_ranges']['J1']['vmax'], 50.)
        self.assertEqual(set(state['feedback']), set(NAMES))
        self.assertNotIn('metrics', state)

    def test_quantiles_include_overflow_and_do_not_exceed_maximum(self):
        m = self.ctl.metrics['J1']
        for i in range(100):
            m.sent(10.+(.025 if i>=95 else .001234),10.,False)
        result=m.snapshot(1.)
        self.assertAlmostEqual(result['lateness_p95_ms'],1.24)
        self.assertAlmostEqual(result['lateness_p99_ms'],25.)
        self.assertLessEqual(result['lateness_p99_ms'],result['lateness_max_ms'])

    def test_models_and_communication_config(self):
        for file in ('arm.hardware.template.json', 'simulation.json'):
            cfg = load_config(ROOT/'configs'/file)
            self.assertEqual(readiness(cfg, True), [])
            self.assertEqual([j['model'] for j in cfg['joints']],
                             ['4310_48V', '8009_48V', '4310_48V']+['3507_24V']*4)
            self.assertEqual([j['can_id'] for j in cfg['joints']], list(range(1, 8)))
            self.assertEqual([j['master_id'] for j in cfg['joints']], list(range(0x11, 0x18)))
        cfg['joints'][0]['limits'] = {'q_min': -1}
        self.assertIn('remove legacy limits', '; '.join(readiness(cfg)))


    def test_arm_explicitly_zeros(self):
        self.ctl.set_zero = True
        self.ctl.arm()
        self.assertEqual(sum(code == ZERO for _, code in self.bus.special_history), 7)

    def test_watchdog_is_optional(self):
        self.c['hardware_watchdog_verified'] = False
        self.assertEqual(readiness(self.c, True), [])

    def test_full_lifecycle_and_no_auto_resume(self):
        self.start()
        self.cycle(20)
        self.assertTrue(all(m.tx >= 19 for m in self.ctl.metrics.values()))
        self.ctl.stop()
        self.assertEqual(self.ctl.state, 'IDLE')
        self.assertTrue(self.ctl.stop_confirmed)
        sent = self.bus.sent
        self.ctl.step()
        self.assertEqual(self.bus.sent, sent)
        with self.assertRaises(SafetyError): self.ctl.submit(self.cmd())

    def test_partial_initialization_failure_stops_every_motor(self):
        self.bus.registers['J3'][22] = float('nan')
        with self.assertRaises(SafetyError): self.ctl.arm()
        self.assertEqual(self.ctl.state, 'FAULT')
        self.assertTrue(self.ctl.stop_confirmed)
        self.assertEqual({n for n, code in self.bus.special_history if code == DISABLE}, set(NAMES))
        self.assertFalse(any(code == ENABLE for _, code in self.bus.special_history))

    def test_command_expiry(self):
        self.start()
        self.clock.now += .051
        self.ctl.step()
        self.assertEqual(self.ctl.state, 'FAULT')
        self.assertIn('expired', self.ctl.reason)
        self.assertTrue(self.ctl.stop_confirmed)

    def test_invalid_commands_stop_group(self):
        for kind in ('missing', 'nan', 'old', 'limit', 'future'):
            with self.subTest(kind=kind):
                self.setUp(); self.start()
                cmd = self.cmd()
                cmd['timestamp'] += .0001
                self.clock.now += .0001
                if kind == 'missing': del cmd['joints']['J2']
                if kind == 'nan': cmd['joints']['J1']['kp'] = float('nan')
                if kind == 'old': cmd['seq'] = 0
                if kind == 'limit': cmd['joints']['J1']['dq_des'] = 51
                if kind == 'future': cmd['timestamp'] += 10
                with self.assertRaises(SafetyError): self.ctl.submit(cmd)
                self.assertEqual(self.ctl.state, 'FAULT')
                self.assertTrue(self.ctl.stop_confirmed)

    def test_initial_algorithm_target_accepted(self):
        self.ctl.arm()
        cmd = self.cmd()
        cmd['joints']['J1'].update(q_des=.3, dq_des=.5, tau_ff=.5)
        self.ctl.submit(cmd)
        self.assertEqual(self.ctl.state, 'RUNNING')
        self.assertTrue(any(code == ENABLE for _, code in self.bus.special_history))

    def test_motion_constraints_and_slew_belong_to_algorithm(self):
        self.ctl.arm()
        cmd = self.cmd()
        # 超过旧位置/速度/力矩/增益限制，且估算力矩远超旧 tau_max。
        cmd['joints']['J1'].update(q_des=4., dq_des=20., kp=100., kd=2., tau_ff=6.)
        self.ctl.submit(cmd)
        self.clock.now += .001
        cmd = self.cmd()
        cmd['joints']['J1'].update(q_des=-4., dq_des=-20., kp=200., kd=3., tau_ff=-6.)
        self.ctl.submit(cmd)
        self.ctl.step()
        self.assertEqual(self.ctl.state, 'RUNNING')


    def test_enable_feedback_can_be_moving_and_away_from_zero(self):
        self.ctl.set_zero = True
        self.ctl.arm()
        original = self.bus.mit
        def moving(joint, target):
            original(joint, target)
            if self.bus.state[joint['name']]['status'] == 1:
                self.bus.state[joint['name']].update(q=.1, dq=.2)
        self.bus.mit = moving
        self.ctl.submit(self.cmd())
        self.assertEqual(self.ctl.state, 'RUNNING')

    def test_feedback_status_and_invalid_numbers_still_fault(self):
        for field, value in (('status',8),('q',float('nan')),('dq',float('inf')),
                             ('tau',float('-inf')),('mos_temperature',float('nan'))):
            with self.subTest(field=field):
                self.setUp(); self.start()
                self.bus.feedback['J1'][field] = value
                self.ctl.step()
                self.assertEqual(self.ctl.state, 'FAULT')


    def test_feedback_motion_and_temperature_are_telemetry(self):
        self.start()
        self.bus.feedback['J1'].update(q=-4., dq=20., tau=8., mos_temperature=80.)
        self.ctl.command['joints']['J1']['kp'] = 100.
        self.ctl.step()
        self.assertEqual(self.ctl.state, 'RUNNING')

    def test_no_stationary_motion_gate(self):
        original = self.bus.refresh
        def moving(joint):
            self.bus.state[joint['name']]['dq'] = .2
            original(joint)
        self.bus.refresh = moving
        self.ctl.arm()
        self.ctl.submit(self.cmd())
        self.assertEqual(self.ctl.state, 'RUNNING')


    def test_isolated_drops_do_not_stall_forever(self):
        self.start()
        for i in range(70):
            if i in (5,15,25,35): self.bus.drop_next['J1'] = 1
            self.cycle()
        m = self.ctl.metrics['J1']
        self.assertEqual(self.ctl.state, 'RUNNING')
        self.assertGreater(m.tx, 60)
        self.assertGreaterEqual(m.tx-m.rx, 4)

    def test_lost_burst_recovers_and_real_disconnect_latches(self):
        self.start()
        self.bus.drop.add('J1')
        self.cycle(10)
        self.assertEqual(self.ctl.state, 'DEGRADED')
        self.bus.drop.clear()
        self.cycle(30)
        self.assertEqual(self.ctl.state, 'RUNNING')
        self.assertGreater(self.ctl.metrics['J1'].recovery_tx, 0)
        self.bus.drop.add('J1')
        self.cycle(70)
        self.assertEqual(self.ctl.state, 'FAULT')
        self.assertFalse(self.ctl.stop_confirmed)
        self.bus.drop.clear()
        self.ctl.reset_fault()
        self.assertEqual(self.ctl.state, 'IDLE')
        self.assertTrue(all(s['status'] == 0 for s in self.bus.state.values()))

    def test_full_queue_no_false_stop_confirmation(self):
        self.start()
        self.bus.send_error = True
        self.clock.now += .001
        self.ctl.step()
        self.assertEqual(self.ctl.state, 'DEGRADED')
        self.cycle(60)
        self.assertEqual(self.ctl.state, 'FAULT')
        self.assertFalse(self.ctl.stop_confirmed)
        self.assertIn('UNCONFIRMED', self.ctl.reason)

    def test_disable_ignored_does_not_confirm(self):
        self.start()
        self.bus.ignore_disable.add('J2')
        self.ctl.stop()
        self.assertFalse(self.ctl.stop_confirmed)
        self.assertEqual(self.ctl.state, 'FAULT')

    def test_slow_cycle_skips_no_catch_up(self):
        self.start()
        self.cycle()
        self.clock.now += .009
        self.ctl.submit(self.cmd()); self.ctl.step()
        self.assertGreaterEqual(self.ctl.skipped, 7)
        self.assertTrue(all(m.tx == 2 for m in self.ctl.metrics.values()))

    def test_stale_feedback_not_refreshed_by_send(self):
        self.start()
        stamp = self.bus.feedback['J1']['timestamp']
        self.bus.drop.add('J1')
        self.cycle(20)
        self.assertEqual(self.bus.feedback['J1']['timestamp'], stamp)

    def test_slow_dead_logger_never_blocks_producer(self):
        release = threading.Event()
        logger = AsyncLogger(lambda _: release.wait(2))
        begin = time.monotonic()
        for _ in range(1000): logger.offer({'state':'RUNNING'})
        self.assertLess(time.monotonic()-begin, .2)
        self.assertGreater(logger.dropped, 0)
        logger.close(); release.set()
        dead = AsyncLogger(lambda _: (_ for _ in ()).throw(BrokenPipeError()))
        dead.offer({}); time.sleep(.03)
        self.assertEqual(dead.failures, 1)
        dead.close()


class ProtocolTests(unittest.TestCase):
    def setUp(self): self.joints = load_config(ROOT/'configs/simulation.json')['joints']

    def test_all_models_and_bounds(self):
        for j in self.joints:
            frame = pack_mit(j, dict(q_des=0.,dq_des=0.,kp=0.,kd=.1,tau_ff=0.))
            self.assertEqual(len(frame.data),8)
            self.assertEqual(frame.flags & 1,1)
            self.assertEqual(frame.data, bytes.fromhex('7fff7ff0000517ff'))
            with self.assertRaises(ValueError):
                pack_mit(j, dict(q_des=20.,dq_des=0.,kp=0.,kd=0.,tau_ff=0.))

    def test_parameter_does_not_look_like_feedback_and_wrong_id_rejected(self):
        for j in self.joints:
            reply = Frame(j['bus'],j['master_id'],bytes([j['can_id'],0,READ,10,1,0,0,0]))
            self.assertEqual(decode(j,reply), ('parameter',(READ,10,1)))
            f = Frame(j['bus'],j['master_id'],bytes([0x10+j['can_id'],0x7F,0x33,0x7F,0xF7,0xFF,30,31]))
            kind, data = decode(j,f)
            self.assertEqual(kind,'feedback')
            self.assertEqual(data['status'],1)
            self.assertIsNone(decode(j,Frame('wrong',f.can_id,f.data)))

    def test_motor_coordinates_ignore_legacy_transform_fields(self):
        j = dict(self.joints[0],direction=-1,zero_joint=.5)
        frame = pack_mit(j,dict(q_des=0.,dq_des=0.,kp=0.,kd=0.,tau_ff=0.))
        self.assertEqual(frame.data[:2],b'\x7f\xff')
        _, data = decode(j,Frame(j['bus'],j['master_id'],bytes([17,127,255,127,247,255,20,20])))
        self.assertAlmostEqual(data['q'],0.,delta=.001)


if __name__ == '__main__': unittest.main()

class LifecycleBoundaryTests(unittest.TestCase):
    setUp = CoreTests.setUp
    cmd = CoreTests.cmd
    start = CoreTests.start
    def test_disconnect_before_arm_does_not_write_bus(self):
        self.ctl.stop('disconnected', fault=True)
        self.assertEqual(self.bus.sent, 0)
        self.assertIsNone(self.ctl.stop_confirmed)
        self.ctl.reset_fault()
        self.assertEqual(self.bus.sent, 0)
        self.assertEqual(self.ctl.state, 'IDLE')

    def test_telemetry_exception_cannot_prevent_stop(self):
        self.start()
        self.ctl.publish = lambda: (_ for _ in ()).throw(BrokenPipeError())
        self.ctl.stop('test stop')
        self.assertTrue(self.ctl.stop_confirmed)

    def test_final_stop_preserves_original_fault_reason(self):
        self.start()
        self.ctl.stop('feedback timeout J1', fault=True)
        self.ctl.stop('service exit')
        self.assertIn('feedback timeout J1', self.ctl.reason)
        self.assertEqual(self.ctl.state,'FAULT')
