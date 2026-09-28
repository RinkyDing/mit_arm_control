import json
import fcntl
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from mit_arm_control import ArmClient
from mit_arm_control.config import NAMES

ROOT = Path(__file__).resolve().parents[1]


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='mit-arm-test-')
        self.path = self.temp.name+'/control.sock'
        self.result = self.temp.name+'/final.json'
        self.env = dict(os.environ, PYTHONPATH=str(ROOT/'src'))
        self.process = subprocess.Popen([sys.executable,'-m','mit_arm_control','serve',
                       '--config',str(ROOT/'configs/simulation.json'),'--socket',self.path,
                       '--final-state',self.result], env=self.env,
                       stdout=subprocess.PIPE if self._testMethodName == 'test_blocked_stdout_cannot_hang_service' else subprocess.DEVNULL,stderr=subprocess.PIPE)
        if self.process.stdout:
            fcntl.fcntl(self.process.stdout.fileno(), fcntl.F_SETPIPE_SZ, 4096)
        deadline = time.monotonic()+3
        while not Path(self.path).exists():
            if self.process.poll() is not None:
                self.fail(self.process.stderr.read().decode())
            if time.monotonic() > deadline: self.fail('service start timeout')
            time.sleep(.01)

    def tearDown(self):
        self.process.terminate()
        try: self.process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            self.process.kill(); self.process.wait(); self.fail('service hung on shutdown')
        self.process.stderr.close()
        if self.process.stdout:
            self.process.stdout.close()
        self.temp.cleanup()

    def initial(self):
        return {n:dict(q_des=0.,dq_des=0.,kp=2.,kd=.1,tau_ff=0.) for n in NAMES}

    def await_state(self, client, state):
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            s = client.get_state()
            if s['state'] == state: return s
            time.sleep(.01)
        self.fail(f'expected {state}, got {s}')

    def test_simulated_algorithm_example_and_final_report(self):
        result = subprocess.run([sys.executable,str(ROOT/'examples/sim_algorithm.py'),
                                '--socket',self.path,'--duration','0.25'],env=self.env,
                                capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertIn('Stop confirmed: True',result.stdout)
        self.process.terminate(); self.process.wait(timeout=4)
        report=json.loads(Path(self.result).read_text())
        self.assertTrue(report['stop_confirmed'])
        self.assertEqual(set(report['metrics']),set(NAMES))

    def test_owner_observer_and_algorithm_disconnect(self):
        with ArmClient(self.path) as client, ArmClient(self.path,'observer') as observer:
            with self.assertRaises(RuntimeError): ArmClient(self.path).connect()
            with self.assertRaises(RuntimeError): observer._rpc('stop')
            client.arm(supported=True,zero_pose=True)
            client.submit(self.initial())
            self.await_state(observer,'RUNNING')
            client.close()
            s=self.await_state(observer,'FAULT')
            self.assertTrue(s['stop_confirmed'])
            self.assertIn('disconnected',s['reason'])
            self.assertIsNone(self.process.poll())

    def test_expired_algorithm_not_automatic_resume_and_explicit_reset(self):
        with ArmClient(self.path) as client:
            client.arm(supported=True,zero_pose=True)
            client.submit(self.initial())
            s=self.await_state(client,'FAULT')
            self.assertIn('expired',s['reason'])
            self.assertTrue(s['stop_confirmed'])
            with self.assertRaises(RuntimeError): client.submit(self.initial())
            client.reset_fault()
            self.assertEqual(client.get_state()['state'],'IDLE')
            self.assertTrue(all(f['status']==0 for f in client.get_state()['feedback'].values()))

    def test_process_kill_detected_by_service(self):
        script='''
import sys,time
from mit_arm_control import ArmClient
from mit_arm_control.config import NAMES
c=ArmClient(sys.argv[1]).connect()
c.arm(supported=True,zero_pose=True)
x={n:dict(q_des=0.,dq_des=0.,kp=2.,kd=.1,tau_ff=0.) for n in NAMES}
while True:
 c.submit(x)
 time.sleep(.005)
'''
        algorithm=subprocess.Popen([sys.executable,'-c',script,self.path],env=self.env,
                                   stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        try:
            with ArmClient(self.path,'observer') as observer:
                self.await_state(observer,'RUNNING')
                algorithm.kill(); algorithm.wait(timeout=2)
                s=self.await_state(observer,'FAULT')
                self.assertTrue(s['stop_confirmed'])
                self.assertIsNone(self.process.poll())
        finally:
            if algorithm.poll() is None: algorithm.kill(); algorithm.wait()

    def test_blocked_stdout_cannot_hang_service(self):
        # Deliberately never drain the child's pipe. It must still answer IPC,
        # report logging failures, and finish shutdown without a stdout lock hang.
        with ArmClient(self.path,'observer') as observer:
            deadline=time.monotonic()+5
            while time.monotonic()<deadline:
                state=observer.get_state()
                if state.get('log_failures',0)>0:
                    break
                time.sleep(.05)
            self.assertGreater(state.get('log_failures',0),0)
            self.assertEqual(state['state'],'IDLE')
        self.process.terminate()
        self.process.wait(timeout=3)

    def test_shutdown_signal_while_armed_confirms_disable(self):
        with ArmClient(self.path) as client:
            client.arm(supported=True,zero_pose=True)
            client.submit(self.initial())
            self.await_state(client,'RUNNING')
            self.process.send_signal(signal.SIGTERM)
            self.process.wait(timeout=4)
            report=json.loads(Path(self.result).read_text())
            self.assertTrue(report['stop_confirmed'])
            self.assertTrue(all(f['status']==0 for f in report['feedback'].values()))


if __name__=='__main__': unittest.main()
