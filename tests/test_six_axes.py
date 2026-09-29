import unittest
from mit_arm_control.config import load_config, NAMES, readiness
from mit_arm_control.controller import Controller
from mit_arm_control.safety import validate_command, SafetyError
import test_core as core

class SixAxisTests(unittest.TestCase):
    def test_six_axis_lifecycle_and_exact_targets(self):
        config = load_config(core.ROOT/'configs/simulation.json', no_gripper=True)
        self.assertEqual([j['name'] for j in config['joints']], list(NAMES[:-1]))
        self.assertEqual(readiness(config), [])
        clock = core.Clock()
        bus = core.VirtualBus(config['joints'], clock)
        ctl = Controller(config, bus, clock=clock)
        ctl.arm()
        self.assertEqual(ctl.snapshot()['joint_names'], list(NAMES[:-1]))
        targets = {n: dict(q_des=0.,dq_des=0.,kp=2.,kd=.1,tau_ff=0.) for n in NAMES[:-1]}
        command = dict(seq=0,timestamp=clock(),joints=targets)
        ctl.submit(command)
        ctl.step()
        self.assertEqual(ctl.state,'RUNNING')
        self.assertNotIn('gripper', bus.rx_seq)
        for wrong in (dict(targets, gripper=targets['J1']), {n:t for n,t in targets.items() if n!='J1'}):
            with self.assertRaises(SafetyError):
                validate_command(config,dict(command,joints=wrong),clock())
        ctl.stop()
        self.assertTrue(ctl.stop_confirmed)
        self.assertFalse(any(n=='gripper' for n, _ in bus.special_history))

    def test_default_keeps_gripper(self):
        config = load_config(core.ROOT/'configs/simulation.json')
        self.assertEqual([j['name'] for j in config['joints']],list(NAMES))
