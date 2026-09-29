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
