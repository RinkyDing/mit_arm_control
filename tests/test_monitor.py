"""只读监视测试；使用虚拟总线，不连接真实 CAN。"""
import unittest
from unittest.mock import patch
from mit_arm_control.config import load_config
from mit_arm_control.monitor import select_joints, query_loop
from mit_arm_control.protocol import READ
from test_core import ROOT, Clock, VirtualBus


class MonitorTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / 'configs/arm.hardware.template.json')
        self.names = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        self.joints = select_joints(self.config, self.names)
        self.clock = Clock()
        self.bus = VirtualBus(self.joints, self.clock)

    def test_selected_ids_and_raw_coordinates(self):
        self.assertEqual([j['can_id'] for j in self.joints], [5, 3, 2, 4, 1, 6])
        self.assertEqual([j['master_id'] for j in self.joints], [21, 19, 18, 20, 17, 22])
        self.assertTrue(all(j['direction'] == 1 and j['zero_joint'] == 0 for j in self.joints))
        self.assertIsNone(self.config['joints'][0]['direction'])
        with self.assertRaises(ValueError):
            select_joints(self.config, ['J1', 'J1'])

    def test_only_read_operations_and_no_gripper(self):
        lines = []
        with patch.object(self.bus, 'register', wraps=self.bus.register) as register, \
             patch.object(self.bus, 'special') as special, patch.object(self.bus, 'mit') as mit:
            query_loop(self.bus, self.joints, 10., .25, clock=self.clock, emit=lines.append)
        self.assertEqual(len(register.call_args_list), 24)
        self.assertTrue(all(call.args[1] == READ for call in register.call_args_list))
        special.assert_not_called()
        mit.assert_not_called()
        self.assertEqual(set(self.bus.protocol_ranges), set(self.names))
        self.assertTrue(all('gripper' not in line for line in lines))
        self.assertTrue(any('RECENT' in line and 'status=0' in line for line in lines))

    def test_no_feedback_is_explicit_not_fabricated_zero(self):
        self.bus._reply = lambda j: None
        lines = []
        query_loop(self.bus, self.joints, 10., .15, clock=self.clock, emit=lines.append)
        self.assertEqual(sum('NO_FEEDBACK' in line for line in lines), 6)

    def test_invalid_range_aborts_before_status_queries(self):
        self.bus.registers['J2'][21] = float('nan')
        with patch.object(self.bus, 'refresh') as refresh:
            with self.assertRaises(ValueError):
                query_loop(self.bus, self.joints, clock=self.clock, emit=lambda _: None)
        refresh.assert_not_called()
        self.assertEqual(self.bus.protocol_ranges, {})

    def test_zero_then_hand_motion_never_enables_or_sends_mit(self):
        from mit_arm_control.protocol import DISABLE, ZERO, ENABLE
        for state in self.bus.state.values():
            state.update(status=1, q=.5)
        lines = []

        def display(line):
            lines.append(line)
            if line.startswith('ZERO VERIFIED:'):
                # 模拟校准结束后手动转动 J6；控制端没有发送运动目标。
                self.bus.state['J6']['q'] = .25

        with patch.object(self.bus, 'mit') as mit:
            query_loop(self.bus, self.joints, 50., .25, clock=self.clock,
                       emit=display, set_zero=True, report_rate=5.)
        mit.assert_not_called()
        history = self.bus.special_history
        self.assertFalse(any(code == ENABLE for _, code in history))
        self.assertEqual({name for name, code in history if code == DISABLE}, set(self.names))
        self.assertEqual([name for name, code in history if code == ZERO], self.names)
        self.assertTrue(all(state['status'] == 0 for state in self.bus.state.values()))
        self.assertTrue(any('J6 ' in line and 'q=+0.25000' in line for line in lines))

    def test_failed_disable_prevents_zero(self):
        from mit_arm_control.protocol import ZERO
        self.bus.state['J3']['status'] = 1
        self.bus.ignore_disable.add('J3')
        with self.assertRaisesRegex(RuntimeError, 'timeout'):
            query_loop(self.bus, self.joints, clock=self.clock, set_zero=True, emit=lambda _: None)
        self.assertFalse(any(code == ZERO for _, code in self.bus.special_history))

    def test_zero_mismatch_aborts_monitoring(self):
        from mit_arm_control.protocol import ZERO
        self.bus.state['J3']['q'] = .5
        original = self.bus.special

        def ignore_zero(joint, code):
            if joint['name'] == 'J3' and code == ZERO:
                return
            original(joint, code)

        lines = []
        with patch.object(self.bus, 'special', side_effect=ignore_zero):
            with self.assertRaisesRegex(RuntimeError, 'zero verification failed'):
                query_loop(self.bus, self.joints, clock=self.clock, set_zero=True, emit=lines.append)
        self.assertFalse(any(line.startswith('ZERO VERIFIED:') for line in lines))

    def test_zero_requires_explicit_pose_confirmation_before_bus_access(self):
        from mit_arm_control.monitor import run_monitor
        with patch('mit_arm_control.monitor.SocketCAN') as backend:
            with self.assertRaisesRegex(ValueError, 'zero-pose-confirmed'):
                run_monitor(self.config, self.names, set_zero=True)
        backend.assert_not_called()

    def test_motion_during_zeroing_is_rejected(self):
        from mit_arm_control.protocol import DISABLE, ZERO
        original = self.bus.special

        def still_moving(joint, code):
            original(joint, code)
            if code == DISABLE:
                self.bus.state[joint['name']]['dq'] = .5

        with patch.object(self.bus, 'special', side_effect=still_moving):
            with self.assertRaisesRegex(RuntimeError, 'moving during calibration'):
                query_loop(self.bus, self.joints, clock=self.clock, set_zero=True, emit=lambda _: None)
        self.assertFalse(any(code == ZERO for _, code in self.bus.special_history))
