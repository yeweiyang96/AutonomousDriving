"""v4 Python client against the production C state machine; no real USB/network."""
import ctypes
import os
import struct
import unittest
from unittest.mock import patch

from adrive.lease import REQUEST
from adrive.motion_protocol import MotionClient,TimedKeyframe,decode_motion_status
from adrive.pico import PicoError
from tests.test_native_lease import DLL,NativeSocket


class MotionSocket(NativeSocket):
    def send(self,data):
        self.sent.append(data)
        if data[5]==self.drop_request:
            self.drop_request=None;return len(data)
        request=(ctypes.c_uint8*len(data)).from_buffer_copy(data);response=(ctypes.c_uint8*80)()
        self.native.exchange_motion(request,len(data),round(self.now*1000)&0xffffffff,response)
        if not self.drop_reply:self.inbox.append(bytes(response))
        self.drop_reply=False
        return len(data)


@unittest.skipUnless(os.name=='nt' and DLL.exists(),'需要构建生产固件主机 DLL')
class MotionProtocolTests(unittest.TestCase):
    def setUp(self):
        self.native=ctypes.CDLL(str(DLL));self.native.reset()
        self.native.exchange_motion.argtypes=[ctypes.POINTER(ctypes.c_uint8),ctypes.c_uint,ctypes.c_uint32,ctypes.POINTER(ctypes.c_uint8)]
        self.native.tick.argtypes=[ctypes.c_uint32]
        self.socket=MotionSocket(self.native)
        self.patcher=patch('adrive.lease.socket.socket',return_value=self.socket);self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.client=MotionClient();self.addCleanup(self.client.close)
        self.activate()

    def tick(self,ms,complete=True,poll=True):
        self.socket.now=ms/1000;self.native.tick(ms&0xffffffff)
        if complete:self.native.complete_keyboard(ms&0xffffffff,1)
        if poll:self.client.poll(ms/1000)

    def keys(self):
        out=(ctypes.c_uint8*7)();self.native.copy_keyboard(out)
        return tuple(k for k in out[1:] if k)

    def activate(self,start=10000):
        self.socket.now=start/1000;self.client.begin(start/1000)
        for ms in range(start,start+120,10):self.tick(ms)
        self.assertTrue(self.client.is_active)

    def window(self,ms,frames,event=0):
        self.socket.now=ms/1000
        return self.client.set_window(tuple(TimedKeyframe(d,k) for d,k in frames),ms/1000,event_id=event)

    def raw(self,data,ms):
        request=(ctypes.c_uint8*len(data)).from_buffer_copy(data);out=(ctypes.c_uint8*80)()
        self.native.exchange_motion(request,len(data),ms&0xffffffff,out)
        return decode_motion_status(bytes(out),ms/1000)

    def test_w_turn_w_has_no_release_and_separate_usb_completion(self):
        baseline=self.native.keyboard_releases()
        revision=self.window(10120,[(40,(26,)),(40,(26,4)),(100,(26,))])
        self.tick(10121,complete=False);self.assertEqual(self.keys(),(26,))
        self.socket.now=10.122;self.client.renew(10.122);self.client.poll(10.122)
        self.assertEqual(self.client.status.submitted_revision,revision)
        self.assertEqual(self.client.status.completed_revision,0)
        self.native.complete_keyboard(10123,1)
        self.tick(10161);self.assertEqual(self.keys(),(26,4))
        self.tick(10201);self.assertEqual(self.keys(),(26,))
        self.assertEqual(self.native.keyboard_releases(),baseline)

    def test_atomic_replacement_sustains_twenty_seconds_without_zero(self):
        baseline=self.native.keyboard_releases()
        for ms in range(10120,30120,40):
            self.window(ms,[(40,(26,4) if ms%80 else (26,7)),(160,(26,))])
            self.tick(ms+1)
            self.assertIn(26,self.keys())
        self.assertEqual(self.native.keyboard_releases(),baseline)

    def test_duplicate_packet_and_event_never_restart_action(self):
        self.window(10120,[(20,(26,84)),(180,(26,))],event=1)
        packet=self.socket.sent[-1];self.tick(10121);self.tick(10141)
        self.assertEqual(self.keys(),(26,))
        status=self.raw(packet,10142);self.assertEqual(status.result,0)
        self.tick(10143);self.assertEqual(self.keys(),(26,));self.assertEqual(status.start_ms,10121)
        new=bytearray(packet);struct.pack_into('<I',new,28,2);struct.pack_into('<I',new,40,2)
        self.assertEqual(self.raw(new,10144).result,5)

    def test_renewal_does_not_extend_window_or_fake_execution(self):
        self.native.set_endpoint_ready(0)
        self.window(10120,[(200,(26,))]);self.tick(10121)
        self.socket.now=10.18;self.client.renew(10.18);self.client.poll(10.18)
        self.assertEqual(self.client.status.submitted_revision,0)
        self.assertGreater(self.client.status.accepted_seq,0)
        self.native.set_endpoint_ready(1);self.tick(10290,poll=False)
        # Deadline is based on the earlier status timestamp, not receipt/renew.
        self.tick(10320,poll=False);self.assertEqual(self.keys(),())

    def test_usb_backpressure_cancels_instead_of_catching_up(self):
        self.window(10120,[(40,(26,)),(40,(26,4)),(120,(26,))]);self.tick(10121)
        self.native.set_endpoint_ready(0);self.tick(10161,poll=False);self.tick(10167,poll=False)
        self.native.set_endpoint_ready(1);self.tick(10168,poll=False)
        self.assertEqual(self.keys(),())

    def test_delayed_packet_and_old_revision_are_rejected(self):
        self.window(10120,[(200,(26,))]);packet=self.socket.sent[-1];self.tick(10121)
        old=bytearray(packet);struct.pack_into('<I',old,28,2)
        self.assertEqual(self.raw(old,10122).result,5)
        stale=bytearray(old);struct.pack_into('<I',stale,40,2);struct.pack_into('<I',stale,52,10100)
        self.assertEqual(self.raw(stale,10123).result,2)

    def test_stop_wins_over_queued_and_late_windows(self):
        self.window(10120,[(200,(26,))]);packet=self.socket.sent[-1];self.tick(10121)
        self.socket.now=10.13;self.client.stop(10.13);self.tick(10131)
        self.assertEqual(self.keys(),());self.assertNotEqual(self.raw(packet,10132).result,0)

    def test_clock_wrap_keeps_duration(self):
        self.client.stop(10.12);self.native.reset();self.socket.inbox.clear()
        self.activate(0xfffffff0-150)
        start=0xfffffff0-30
        self.window(start,[(40,(26,)),(40,(26,4)),(120,(26,))])
        self.tick(start+1);self.tick(start+41);self.assertEqual(self.keys(),(26,4))
        self.tick(start+81);self.assertEqual(self.keys(),(26,))

    def test_frame_validation_and_capability_are_mandatory(self):
        for frames in ([(19,(26,))],[(201,(26,))],[(100,(26,))]*3,[(20,(26,26))]):
            with self.assertRaises(ValueError):self.window(10120,frames)
        with self.assertRaises(PicoError):decode_motion_status(b'ADRS'+bytes([3])+bytes(75),10)

    def test_wifi_and_usb_loss_invalidate_motion(self):
        for loss in ('set_network','set_usb_available'):
            with self.subTest(loss=loss):
                self.window(10120,[(200,(26,))]);self.tick(10121)
                getattr(self.native,loss)(0);self.tick(10122,poll=False)
                getattr(self.native,loss)(1);self.tick(10123,poll=False)
                self.assertEqual(self.keys(),())
                self.client.stop(10.124);self.native.reset();self.socket.inbox.clear();self.activate()
