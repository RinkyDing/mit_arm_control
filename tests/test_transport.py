"""SocketCAN wire/ancillary validation with mocked sockets only."""
import socket
import struct
import unittest
from unittest.mock import Mock, patch
from mit_arm_control.backend import SocketCAN
from mit_arm_control.config import load_config
from mit_arm_control.protocol import pack_mit
from test_core import ROOT


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.joints=load_config(ROOT/'configs/simulation.json')['joints']
        self.bus=SocketCAN.__new__(SocketCAN)
        self.sock=Mock()
        self.bus.protocol_ranges={}
        self.bus.joints=self.joints
        self.bus.sockets={'can0':self.sock}
        self.bus.feedback={};self.bus.parameters={};self.bus._round_robin=0
        self.bus.rx_seq={j['name']:0 for j in self.joints}
        self.bus.clock=lambda:10.

    def feed(self, payload, identifier=0x11, flags=0, ancillary=True):
        raw=struct.pack('=IBBBB64s',identifier,len(payload),5,0,0,payload.ljust(64,b'\0'))
        anc=[(socket.SOL_SOCKET,getattr(socket,'SO_TIMESTAMPNS',35),struct.pack('@ll',100,0))] if ancillary else []
        self.sock.recvmsg.return_value=(raw,anc,flags,None)
        with patch('mit_arm_control.backend.select.select',return_value=([self.sock],[],[])), \
             patch('mit_arm_control.backend.time.time',return_value=100.02):
            self.bus.poll()

    def test_fd_brs_frame_layout(self):
        frame=pack_mit(self.joints[0],dict(q_des=0.,dq_des=0.,kp=0.,kd=.1,tau_ff=0.))
        self.sock.send.return_value=72
        self.bus.send(frame)
        raw=self.sock.send.call_args.args[0]
        self.assertEqual(len(raw),72)
        self.assertEqual(raw[4:6],bytes([8,1]))
        self.assertEqual(raw[8:16],frame.data)

    def test_kernel_timestamp_not_processing_time(self):
        self.feed(bytes([17,127,255,127,247,255,20,20]))
        self.assertAlmostEqual(self.bus.feedback['J1']['timestamp'],9.98)
        self.assertEqual(self.bus.rx_seq['J1'],1)

    def test_late_parameters_never_refresh_motor_timestamp(self):
        self.feed(bytes([1,0,0x33,10,1,0,0,0]))
        self.assertEqual(self.bus.feedback,{})
        self.assertEqual(self.bus.parameters[('J1',0x33,10)][1],1)

    def test_invalid_frames_cannot_refresh_liveness(self):
        data=bytes([17,127,255,127,247,255,20,20])
        self.feed(data,ancillary=False)
        self.feed(data,flags=socket.MSG_CTRUNC)
        self.feed(data,identifier=0x20000011)
        self.feed(data,identifier=0x12)
        self.assertEqual(self.bus.feedback,{})
