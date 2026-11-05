"""SocketCAN wire/ancillary validation with mocked sockets only."""
import socket
import struct
import unittest
from unittest.mock import Mock, patch
from mit_arm_control.backend import SocketCAN, transport_counters, SO_RXQ_OVFL, CAN_RAW_ERR_FILTER
from mit_arm_control.config import load_config
from mit_arm_control.protocol import pack_mit
from test_core import ROOT


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.joints=load_config(ROOT/'configs/simulation.json')['joints']
        self.bus=SocketCAN.__new__(SocketCAN)
        self.sock=Mock()
        self.bus.protocol_ranges={}
        self.bus._mit_cache={}
        self.bus.last_feedback_name=None
        self.bus.feedback_revision=0
        self.bus.transport={'can0':transport_counters()}
        self.bus.joints=self.joints
        self.bus.sockets={'can0':self.sock}
        self.bus._socket_list=(self.sock,)
        self.bus._bus_by_socket={self.sock:'can0'}
        self.bus._joint_by_id={(j['bus'],j['master_id']):j for j in self.joints}
        self.bus.feedback={};self.bus.parameters={};self.bus._round_robin=0
        self.bus.rx_seq={j['name']:0 for j in self.joints}
        self.bus.clock=lambda:10.

    def feed(self, payload, identifier=0x11, flags=0, ancillary=True):
        raw=struct.pack('=IBBBB64s',identifier,len(payload),5,0,0,payload.ljust(64,b'\0'))
        anc=[(socket.SOL_SOCKET,getattr(socket,'SO_TIMESTAMPNS',35),struct.pack('@ll',100,0))] if ancillary else []
        self.sock.recvmsg.return_value=(raw,anc,flags,None)
        with patch('mit_arm_control.backend.time.time',return_value=100.02):
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

    def test_nonblocking_poll_never_selects_or_waits(self):
        with patch('mit_arm_control.backend.select.select',side_effect=AssertionError('unexpected wait')):
            self.feed(bytes([23,127,255,127,247,255,20,20]), identifier=0x17)
            self.assertEqual(self.bus.rx_seq['gripper'],1)
            self.sock.recvmsg.side_effect=BlockingIOError()
            self.assertFalse(self.bus.poll())

    def test_empty_first_bus_does_not_hide_second_bus(self):
        other=Mock()
        self.bus._socket_list=(self.sock,other)
        self.bus._bus_by_socket[other]='can1'
        joint=dict(self.joints[0],bus='can1')
        self.bus._joint_by_id[('can1',0x11)]=joint
        self.sock.recvmsg.side_effect=BlockingIOError()
        data=bytes([17,127,255,127,247,255,20,20])
        raw=struct.pack('=IBBBB64s',0x11,8,5,0,0,data)
        ancillary=[(socket.SOL_SOCKET,getattr(socket,'SO_TIMESTAMPNS',35),struct.pack('@ll',100,0))]
        other.recvmsg.return_value=(raw,ancillary,0,None)
        with patch('mit_arm_control.backend.time.time',return_value=100.02):
            self.assertTrue(self.bus.poll())
        self.assertEqual(self.bus.rx_seq['J1'],1)

    def test_poll_propagates_network_error(self):
        self.sock.recvmsg.side_effect=OSError(100,'network down')
        with self.assertRaises(OSError):
            self.bus.poll()

    def test_transaction_wait_timeout_does_not_read(self):
        with patch('mit_arm_control.backend.select.select',return_value=([],[],[])) as select:
            self.assertFalse(self.bus.poll(.002))
        select.assert_called_once_with(self.bus._socket_list,[],[],.002)
        self.sock.recvmsg.assert_not_called()

    def test_cached_queries_keep_wire_format_and_send_errors(self):
        from mit_arm_control.protocol import register_request, STATUS
        with patch('mit_arm_control.backend.socket.socket',return_value=self.sock):
            bus=SocketCAN(self.joints)
        self.sock.send.return_value=72
        for j in self.joints:
            bus.refresh(j)
            self.assertEqual(self.sock.send.call_args.args[0],bus.encode(register_request(j,STATUS)))
        self.sock.send.side_effect=BlockingIOError(11,'try again')
        with self.assertRaises(BlockingIOError):
            bus.refresh(self.joints[0])

    def test_parameter_or_empty_read_never_reuses_feedback_notification(self):
        self.feed(bytes([17,127,255,127,247,255,20,20]))
        self.assertEqual(self.bus.last_feedback_name,'J1')
        self.assertEqual(self.bus.feedback_revision,1)
        self.feed(bytes([1,0,0x33,10,1,0,0,0]))
        self.assertIsNone(self.bus.last_feedback_name)
        self.assertEqual(self.bus.feedback_revision,1)
        self.sock.recvmsg.side_effect=BlockingIOError()
        self.assertFalse(self.bus.poll())
        self.assertIsNone(self.bus.last_feedback_name)

    def test_error_frame_updates_transport_only_without_timestamp(self):
        self.feed(bytes(8), identifier=0x20000060, ancillary=False)
        self.assertEqual(self.bus.transport['can0']['bus_off'],1)
        self.assertEqual(self.bus.transport['can0']['ack_errors'],1)
        self.assertEqual(self.bus.transport['can0']['error_frames'],1)
        self.assertEqual(self.bus.feedback,{})
        self.assertEqual(self.bus.feedback_revision,0)
        self.assertIsNone(self.bus.last_feedback_name)

    def test_kernel_overflow_counter_does_not_refresh_feedback(self):
        ancillary=[(socket.SOL_SOCKET,SO_RXQ_OVFL,struct.pack('=I',19))]
        self.sock.recvmsg.return_value=(bytes(16),ancillary,0,None)
        self.bus.poll()
        self.assertEqual(self.bus.transport['can0']['rx_queue_dropped'],19)
        self.assertEqual(self.bus.feedback,{})

    def test_socket_filters_reject_other_ids_extended_and_remote_frames(self):
        with patch('mit_arm_control.backend.socket.socket',return_value=self.sock):
            bus=SocketCAN(self.joints)
        options={(c.args[0],c.args[1]):c.args[2] for c in self.sock.setsockopt.call_args_list}
        filters=list(struct.iter_unpack('=II',options[(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER)]))
        def accepts(can_id):
            return any(can_id & mask == ident & mask for ident,mask in filters)
        self.assertTrue(all(accepts(j['master_id']) for j in self.joints))
        self.assertFalse(accepts(0x7ff))
        self.assertFalse(accepts(0x80000011))
        self.assertFalse(accepts(0x40000011))
        self.assertIn((socket.SOL_CAN_RAW,CAN_RAW_ERR_FILTER),options)
        bus.close()

    def test_mit_cache_tracks_mutated_values_ranges_and_rejects_invalid(self):
        joint=self.joints[3]
        target=dict(q_des=1.,dq_des=0.,kp=0.,kd=.1,tau_ff=0.)
        self.sock.send.return_value=72
        self.bus.mit(joint,target)
        first=self.sock.send.call_args.args[0]
        target['q_des']=2.
        self.bus.mit(joint,target)
        self.assertNotEqual(first,self.sock.send.call_args.args[0])
        self.bus.protocol_ranges[joint['name']]=(12.566,50.,5.)
        self.bus.mit(joint,target)
        expected=self.bus.encode(pack_mit(joint,target,(12.566,50.,5.)))
        self.assertEqual(expected,self.sock.send.call_args.args[0])
        for value in (float('nan'),float('inf'),13.):
            target['q_des']=value
            before=self.sock.send.call_count
            with self.assertRaises(ValueError):
                self.bus.mit(joint,target)
            self.assertEqual(self.sock.send.call_count,before)

    def test_cached_mit_still_propagates_congestion_and_short_write(self):
        j=self.joints[0]
        target=dict(q_des=0.,dq_des=0.,kp=0.,kd=.1,tau_ff=0.)
        self.sock.send.return_value=72
        self.bus.mit(j,target)
        self.sock.send.side_effect=BlockingIOError(11,'queue full')
        with self.assertRaises(BlockingIOError): self.bus.mit(j,target)
        self.sock.send.side_effect=None
        self.sock.send.return_value=16
        with self.assertRaises(OSError): self.bus.mit(j,target)
