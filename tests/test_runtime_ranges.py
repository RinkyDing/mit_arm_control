"""Runtime protocol ranges: simulated lifecycle and mocked CAN, no hardware."""
import copy
import unittest
from unittest.mock import patch
import test_core as core
import test_transport as transport
from mit_arm_control.config import MODELS, readiness
from mit_arm_control.protocol import Frame, WRITE, ENABLE, ZERO, decode, pack_mit
from mit_arm_control.safety import SafetyError


class RuntimeRangeTests(unittest.TestCase):
    setUp = core.CoreTests.setUp
    cmd = core.CoreTests.cmd

    def test_actual_ranges_are_per_axis_and_do_not_modify_config_or_motor(self):
        original = copy.deepcopy(self.c)
        self.bus.registers['J1'].update({21: 8., 22: 30., 23: 6.})
        self.bus.registers['J4'].update({21: 10., 22: 40., 23: 8.})
        with patch.object(self.bus, 'register', wraps=self.bus.register) as register:
            self.ctl.arm(True, True)
        self.assertEqual(self.c, original)
        self.assertEqual(MODELS['4310_48V'], (12.5, 50., 10.))
        self.assertEqual(self.bus.protocol_ranges['J1'], (8., 30., 6.))
        self.assertEqual(self.bus.protocol_ranges['J4'], (10., 40., 8.))
        self.assertEqual(self.ctl.snapshot()['protocol_ranges']['J1']['vmax'], 30.)
        writes = [call.args[2] for call in register.call_args_list if call.args[1] == WRITE]
        self.assertEqual(writes, [10] * 7)
        self.ctl.submit(self.cmd())
        self.assertEqual(self.ctl.state, 'RUNNING')

    def test_invalid_ranges_abort_before_zero_or_enable(self):
        for value in (0., -1., float('nan'), float('inf')):
            with self.subTest(value=value):
                self.setUp()
                self.bus.registers['J3'][21] = value
                with self.assertRaises(SafetyError):
                    self.ctl.arm(True, True)
                self.assertEqual(self.ctl.state, 'FAULT')
                self.assertTrue(self.ctl.stop_confirmed)
                self.assertEqual(self.ctl.protocol_ranges, {})
                self.assertEqual(self.bus.protocol_ranges, {})
                self.assertFalse(any(code in (ENABLE, ZERO) for _, code in self.bus.special_history))

    def test_each_limit_must_fit_actual_ranges(self):
        for rid, value in ((21, 1.), (22, 1.), (23, 1.)):
            with self.subTest(rid=rid):
                self.setUp()
                original = copy.deepcopy(self.c)
                self.bus.registers['J3'][rid] = value
                with self.assertRaisesRegex(SafetyError, 'configured limits exceed'):
                    self.ctl.arm(True, True)
                self.assertEqual(self.c, original)
                self.assertEqual(self.bus.protocol_ranges, {})
                self.assertTrue(self.ctl.stop_confirmed)

    def test_failed_read_cannot_partially_apply_ranges(self):
        self.bus.registers['J1'][22] = 30.
        self.bus.drop.add('J3')
        with self.assertRaises(TimeoutError):
            self.ctl.arm(True, True)
        self.assertEqual(self.bus.protocol_ranges, {})
        self.assertEqual(self.ctl.protocol_ranges, {})
        self.assertFalse(self.ctl.stop_confirmed)
        self.assertFalse(any(code == ENABLE for _, code in self.bus.special_history))

    def test_rearm_rereads_ranges(self):
        self.ctl.arm(True, True)
        self.ctl.stop()
        self.bus.registers['J1'][22] = 35.
        self.ctl.arm(True, True)
        self.assertEqual(self.bus.protocol_ranges['J1'][1], 35.)
        self.assertEqual(self.ctl.snapshot()['protocol_ranges']['J1']['vmax'], 35.)

    def test_hardware_offline_check_defers_actual_capacity_check(self):
        self.c['hardware_commissioned'] = True
        self.c['joints'][0]['limits']['dq_max'] = 60.
        self.assertEqual(readiness(self.c, True), [])
        ranges = {j['name']: MODELS[j['model']] for j in self.c['joints']}
        self.assertTrue(readiness(self.c, True, ranges=ranges))
        ranges['J1'] = (12.5, 70., 10.)
        self.assertEqual(readiness(self.c, True, ranges=ranges), [])

    def test_codec_uses_custom_ranges_for_both_directions(self):
        joint = dict(self.c['joints'][0], direction=-1, zero_joint=.5)
        target = dict(q_des=-5.5, dq_des=-15., kp=0., kd=0., tau_ff=-4.)
        frame = pack_mit(joint, target, (6., 15., 4.))
        self.assertEqual(frame.data[:3], bytes([255, 255, 255]))
        self.assertEqual(frame.data[3] >> 4, 15)
        self.assertEqual(frame.data[6] & 15, 15)
        self.assertEqual(frame.data[7], 255)
        reply = Frame(joint['bus'], joint['master_id'], bytes([17,255,255,255,255,255,25,25]))
        _, actual = decode(joint, reply, (6., 15., 4.))
        self.assertEqual((actual['q'], actual['dq'], actual['tau']), (-5.5, -15., -4.))

    def test_socketcan_uses_runtime_ranges_in_tx_and_rx(self):
        fixture = transport.TransportTests()
        fixture.setUp()
        bus, joint = fixture.bus, fixture.joints[0]
        bus.protocol_ranges['J1'] = (6., 15., 4.)
        fixture.sock.send.return_value = 72
        bus.mit(joint, dict(q_des=6., dq_des=15., kp=0., kd=0., tau_ff=4.))
        raw = fixture.sock.send.call_args.args[0]
        self.assertEqual(raw[8:11], bytes([255,255,255]))
        fixture.feed(bytes([17,255,255,255,255,255,25,25]))
        actual = bus.feedback['J1']
        self.assertEqual((actual['q'], actual['dq'], actual['tau']), (6.,15.,4.))
