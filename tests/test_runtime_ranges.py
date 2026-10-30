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
            self.ctl.arm()
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
                    self.ctl.arm()
                self.assertEqual(self.ctl.state, 'FAULT')
                self.assertTrue(self.ctl.stop_confirmed)
                self.assertEqual(self.ctl.protocol_ranges, {})
                self.assertEqual(self.bus.protocol_ranges, {})
                self.assertFalse(any(code in (ENABLE, ZERO) for _, code in self.bus.special_history))

    def test_small_actual_ranges_replace_defaults_without_motion_limits(self):
        for rid in (21, 22, 23):
            with self.subTest(rid=rid):
                self.setUp()
                self.bus.registers['J3'][rid] = 1.
                self.ctl.arm()
                self.assertEqual(self.ctl.protocol_ranges['J3'][rid-21], 1.)
                self.ctl.submit(self.cmd())
                self.assertEqual(self.ctl.state, 'RUNNING')


    def test_failed_read_cannot_partially_apply_ranges(self):
        self.bus.registers['J1'][22] = 30.
        self.bus.drop.add('J3')
        with self.assertRaises(TimeoutError):
            self.ctl.arm()
        self.assertEqual(self.bus.protocol_ranges, {})
        self.assertEqual(self.ctl.protocol_ranges, {})
        self.assertFalse(self.ctl.stop_confirmed)
        self.assertFalse(any(code == ENABLE for _, code in self.bus.special_history))

    def test_rearm_rereads_ranges(self):
        self.ctl.arm()
        self.ctl.stop()
        self.bus.registers['J1'][22] = 35.
        self.ctl.arm()
        self.assertEqual(self.bus.protocol_ranges['J1'][1], 35.)
        self.assertEqual(self.ctl.snapshot()['protocol_ranges']['J1']['vmax'], 35.)

    def test_entire_group_is_checked_against_actual_ranges_before_enable(self):
        for field, value in (('q_des', 6.1), ('dq_des', 15.1), ('tau_ff', 4.1),
                             ('kp', 500.1), ('kd', 5.1), ('kp', -.1), ('kd', -.1)):
            with self.subTest(field=field):
                self.setUp()
                self.bus.registers['gripper'].update({21: 6., 22: 15., 23: 4.})
                self.ctl.arm()
                cmd = self.cmd()
                cmd['joints']['gripper'][field] = value
                with patch.object(self.bus, 'mit', wraps=self.bus.mit) as mit:
                    with self.assertRaisesRegex(SafetyError, 'protocol range'):
                        self.ctl.submit(cmd)
                mit.assert_not_called()
                self.assertFalse(any(code == ENABLE for _, code in self.bus.special_history))

    def test_gripper_runtime_range_can_exceed_model_default(self):
        self.bus.registers['gripper'].update({21: 14., 22: 120., 23: 6.})
        self.ctl.arm()
        cmd = self.cmd()
        cmd['joints']['gripper'].update(q_des=13., dq_des=110., tau_ff=5.5, kp=0., kd=0.)
        self.ctl.submit(cmd)
        self.assertEqual(self.ctl.state, 'RUNNING')
        self.assertEqual(MODELS['3507_24V'], (12.566, 100., 5.))
        self.assertEqual(self.ctl.snapshot()['protocol_ranges']['gripper']['vmax'], 120.)

    def test_runtime_range_validation(self):
        self.assertEqual(readiness(self.c, True), [])
        ranges = {j['name']: MODELS[j['model']] for j in self.c['joints']}
        self.assertEqual(readiness(self.c, True, ranges=ranges), [])
        for invalid in (None, (1., 0., 1.), (1., float('inf'), 1.)):
            ranges['gripper'] = invalid
            self.assertTrue(readiness(self.c, True, ranges=ranges))


    def test_codec_uses_custom_ranges_without_coordinate_transform(self):
        joint = dict(self.c['joints'][0], direction=-1, zero_joint=.5)
        target = dict(q_des=6., dq_des=15., kp=0., kd=0., tau_ff=4.)
        frame = pack_mit(joint, target, (6., 15., 4.))
        self.assertEqual(frame.data[:3], bytes([255, 255, 255]))
        self.assertEqual(frame.data[3] >> 4, 15)
        self.assertEqual(frame.data[6] & 15, 15)
        self.assertEqual(frame.data[7], 255)
        reply = Frame(joint['bus'], joint['master_id'], bytes([17,255,255,255,255,255,25,25]))
        _, actual = decode(joint, reply, (6., 15., 4.))
        self.assertEqual((actual['q'], actual['dq'], actual['tau']), (6., 15., 4.))

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


class IPCRuntimeRangeTests(unittest.TestCase):
    setUp = core.CoreTests.setUp
    cmd = core.CoreTests.cmd

    def test_ipc_uses_actual_ranges_instead_of_model_defaults(self):
        from mit_arm_control.ipc import IPCServer, Mailbox
        self.bus.registers['gripper'].update({21: 14., 22: 120., 23: 6.})
        self.bus.registers['J6'][22] = 10.
        self.ctl.arm()
        # 直接调用 IPC 分发，避免为了测试校验逻辑再创建服务／总线。
        ipc = IPCServer.__new__(IPCServer)
        client = object()
        ipc.config, ipc.owner, ipc.clients = self.c, client, {client: 'control'}
        ipc.previous = None
        ipc.mailbox = Mailbox()
        ipc.mailbox.snapshot = self.ctl.snapshot()
        cmd = self.cmd()
        cmd['joints']['gripper'].update(q_des=13., dq_des=110., tau_ff=5.5, kp=0., kd=0.)
        with patch('mit_arm_control.ipc.time.monotonic', self.clock):
            result = ipc.dispatch(client, dict(version=1, id=1, op='submit', command=cmd))
            self.assertTrue(result['queued'])
            self.ctl.submit(ipc.mailbox.take_command())
            self.assertEqual(self.ctl.state, 'RUNNING')
            self.clock.now += .001
            cmd = self.cmd()
            cmd['joints']['J6']['dq_des'] = 11.
            with self.assertRaisesRegex(SafetyError, 'protocol range'):
                ipc.dispatch(client, dict(version=1, id=2, op='submit', command=cmd))
            self.assertIsNone(ipc.mailbox.take_command())
            self.assertTrue(ipc.mailbox.stop.is_set())

    def test_reserved_mit_payload_rejected_before_any_enable(self):
        self.ctl.arm()
        cmd = self.cmd()
        # 全高位 + 0xFE 会被驱动解释为设零，必须拒绝这种运动数据。
        cmd['joints']['gripper'] = dict(q_des=12.566, dq_des=100., kp=500., kd=5.,
                                      tau_ff=4.998)
        with patch.object(self.bus, 'mit', wraps=self.bus.mit) as mit:
            with self.assertRaisesRegex(SafetyError, 'reserved special command'):
                self.ctl.submit(cmd)
        mit.assert_not_called()
        self.assertFalse(any(code in (ENABLE, ZERO) for _, code in self.bus.special_history))
