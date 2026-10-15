import unittest
from unittest.mock import patch
from mit_arm_control.config import load_config, NAMES
from mit_arm_control.controller import Controller
import test_core as core

class SendOrderTests(unittest.TestCase):
    def setup_controller(self, observation=False, gripper=False):
        c = load_config(core.ROOT/'configs/simulation.json', no_gripper=not gripper)
        for j, can_id in zip(c['joints'], (5,3,2,4,1,6,7)):
            j.update(can_id=can_id, master_id=can_id+0x10)
        clock = core.Clock()
        bus = core.VirtualBus(c['joints'], clock)
        ctl = Controller(c, bus, clock=clock, observation_only=observation)
        return c,clock,bus,ctl

    def test_motion_order_and_name_mapping(self):
        c,clock,bus,ctl = self.setup_controller(gripper=True)
        ctl.arm()
        targets={j['name']:dict(q_des=i*.01,dq_des=0.,kp=0.,kd=.1,tau_ff=0.)
                 for i,j in enumerate(c['joints'])}
        ctl.submit(dict(seq=0,timestamp=clock(),joints=targets))
        with patch.object(bus,'mit',wraps=bus.mit) as send:
            ctl.step()
        expected=['J5','J3','J2','J4','J1','J6','gripper']
        self.assertEqual([call.args[0]['name'] for call in send.call_args_list],expected)
        for call in send.call_args_list:
            self.assertEqual(call.args[1], targets[call.args[0]['name']])
        self.assertEqual(ctl.snapshot()['joint_names'],list(NAMES))
        self.assertEqual([j['name'] for j in c['joints']],list(NAMES))

    def test_observation_order(self):
        c,clock,bus,ctl = self.setup_controller(observation=True)
        ctl.observe()
        with patch.object(bus,'refresh',wraps=bus.refresh) as send:
            ctl.step()
        self.assertEqual([call.args[0]['name'] for call in send.call_args_list],
                         ['J5','J3','J2','J4','J1','J6'])
        self.assertEqual(ctl.snapshot()['joint_names'],list(NAMES[:-1]))

    def test_priority_uses_reply_id_not_command_id(self):
        c,clock,bus,_ = self.setup_controller()
        c['joints'][0]['master_id']=0x10
        ctl=Controller(c,bus,clock=clock)
        self.assertEqual(ctl.send_joints[0]['name'],'J1')
