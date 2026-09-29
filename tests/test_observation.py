"""Observation reuses Controller and SDK; all tests use simulation only."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from mit_arm_control import ArmClient
from mit_arm_control.config import load_config, select_joints
from mit_arm_control.controller import Controller
from mit_arm_control.protocol import READ, ZERO, ENABLE, WRITE
from mit_arm_control.safety import SafetyError
import test_core as core


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(core.ROOT / 'configs/arm.hardware.template.json')
        self.names = ['J1','J2','J3','J4','J5','J6']
        self.config = dict(self.config, joints=select_joints(self.config, self.names))
        self.clock = core.Clock()
        self.bus = core.VirtualBus(self.config['joints'], self.clock)
        self.ctl = Controller(self.config, self.bus, clock=self.clock, observation_only=True)

    def cycle(self, n):
        for _ in range(n):
            self.ctl.step()
            self.clock.now += .001

    def test_no_zero_is_read_only_and_rejects_motion_even_when_degraded(self):
        with patch.object(self.bus, 'register', wraps=self.bus.register) as register:
            self.ctl.observe()
        self.cycle(20)
        self.assertEqual(self.ctl.state, 'OBSERVING')
        self.assertTrue(all(call.args[1] == READ for call in register.call_args_list))
        self.assertEqual(self.bus.special_history, [])
        for state in ('OBSERVING','DEGRADED','READY'):
            self.ctl.state = state
            with self.assertRaises(SafetyError): self.ctl.submit({})
            with self.assertRaises(SafetyError): self.ctl.arm(True, True)
        self.ctl.stop()
        self.assertIsNone(self.ctl.stop_confirmed)
        self.assertEqual(self.bus.special_history, [])

    def test_zero_shared_lifecycle_never_enables_or_changes_mode(self):
        with patch.object(self.bus, 'register', wraps=self.bus.register) as register, \
             patch.object(self.bus, 'mit') as mit:
            self.ctl.observe(set_zero=True, workspace_ready=True, zero_pose=True)
            self.bus.state['J6']['q'] = .25
            self.cycle(20)
            self.assertEqual(self.bus.feedback['J6']['q'], .25)
            self.ctl.stop()
        self.assertTrue(self.ctl.stop_confirmed)
        mit.assert_not_called()
        self.assertFalse(any(call.args[1] == WRITE for call in register.call_args_list))
        self.assertEqual([n for n, code in self.bus.special_history if code == ZERO],self.names)
        self.assertFalse(any(code == ENABLE for _, code in self.bus.special_history))

    def test_confirmation_and_failed_zero(self):
        with self.assertRaises(SafetyError): self.ctl.observe(set_zero=True)
        self.assertEqual(self.bus.sent, 0)
        self.bus.state['J3']['q'] = .5
        original = self.bus.special
        def ignore_zero(j, code):
            if j['name'] != 'J3' or code != ZERO: original(j, code)
        with patch.object(self.bus, 'special', side_effect=ignore_zero):
            with self.assertRaises(SafetyError):
                self.ctl.observe(True, True, True)
        self.assertEqual(self.ctl.state,'FAULT')
        self.assertTrue(self.ctl.stop_confirmed)

    def test_shared_backpressure_recovers_then_timeout_latches(self):
        self.ctl.observe()
        self.cycle(5)
        self.bus.drop.add('J1')
        self.cycle(10)
        self.assertEqual(self.ctl.state,'DEGRADED')
        self.bus.drop.clear()
        self.cycle(20)
        self.assertEqual(self.ctl.state,'OBSERVING')
        self.assertGreater(self.ctl.metrics['J1'].recovery_tx,0)
        self.bus.drop.add('J1')
        self.cycle(60)
        self.assertEqual(self.ctl.state,'FAULT')
        self.assertIn('feedback timeout',self.ctl.reason)
        self.bus.drop.clear()
        self.ctl.reset_fault()
        self.assertEqual(self.ctl.state,'IDLE')

    def test_shared_absolute_schedule_skips_without_catchup(self):
        self.ctl.observe()
        self.cycle(2)
        before = self.ctl.metrics['J1'].tx
        self.clock.now += .01
        self.ctl.step()
        self.assertEqual(self.ctl.metrics['J1'].tx, before+1)
        self.assertGreater(self.ctl.skipped, 8)


class ObservationProcessTests(unittest.TestCase):
    def test_sdk_and_existing_example_use_one_service_without_submit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = directory+'/service.sock'
            output = directory+'/final.json'
            env = dict(os.environ,PYTHONPATH=str(core.ROOT/'src'),PYTHONDONTWRITEBYTECODE='1')
            p = subprocess.Popen([sys.executable,'-m','mit_arm_control','serve',
                '--config',str(core.ROOT/'configs/arm.hardware.template.json'),
                '--observe-only','--query-rate','1000','--socket',path,'--final-state',output],
                env=env,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
            try:
                deadline=time.monotonic()+3
                while not Path(path).exists():
                    self.assertIsNone(p.poll())
                    self.assertLess(time.monotonic(),deadline)
                    time.sleep(.01)
                with ArmClient(path) as client:
                    self.assertTrue(client.get_state()['observation_only'])
                    with self.assertRaises(RuntimeError): client.arm(workspace_ready=True,zero_pose=True)
                    client.observe()
                    with self.assertRaises(RuntimeError): client.submit({})
                    time.sleep(.03)
                    state=client.get_state()
                    self.assertEqual(set(state['metrics']),{'J1','J2','J3','J4','J5','J6'})
                    self.assertTrue(all(m['rx']>0 for m in state['metrics'].values()))
                    client.stop()
                # 新服务会话验证已有示例入口；不自动复位断联故障。
                p.terminate();p.wait(timeout=4);p.stderr.close()
                p=subprocess.Popen([sys.executable,'-m','mit_arm_control','serve',
                    '--config',str(core.ROOT/'configs/arm.hardware.template.json'),
                    '--observe-only','--socket',path,'--final-state',output],
                    env=env,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
                deadline=time.monotonic()+3
                while not Path(path).exists():
                    self.assertLess(time.monotonic(),deadline);time.sleep(.01)
                result=subprocess.run([sys.executable,str(core.ROOT/'examples/sim_algorithm.py'),
                    '--observe','--set-zero','--zero-pose-confirmed','--duration','.1',
                    '--print-rate','20','--socket',path],env=env,capture_output=True,text=True,timeout=10)
                self.assertEqual(result.returncode,0,result.stdout+result.stderr)
                self.assertIn('OBSERVING:',result.stdout)
                self.assertIn('Stop confirmed: True',result.stdout)
            finally:
                p.terminate()
                try:p.wait(timeout=4)
                except subprocess.TimeoutExpired:p.kill();p.wait()
                p.stderr.close()
            final=json.loads(Path(output).read_text())
            self.assertTrue(final['observation_only'])
            self.assertTrue(final['stop_confirmed'])
