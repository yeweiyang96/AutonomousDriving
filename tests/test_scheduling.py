"""Native waits and local UDP only: no game window, device, or input action."""
import os
import socket
import threading
import time
import unittest

from adrive.scheduling import LoopWaiter


class WaiterTests(unittest.TestCase):
    def setUp(self):
        self.waiter = LoopWaiter()
        self.addCleanup(self.waiter.close)

    def test_signal_before_wait_is_not_lost(self):
        self.waiter.wake()
        self.assertEqual(self.waiter.wait(2), 'signal')
        self.assertEqual(self.waiter.wait(.003), 'timer')

    def test_signal_interrupts_a_long_wait(self):
        started = threading.Event()
        result = []
        def run():
            started.set()
            result.append(self.waiter.wait(5))
        thread = threading.Thread(target=run)
        thread.start()
        try:
            self.assertTrue(started.wait(1))
            self.waiter.wake()
            thread.join(1)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result, ['signal'])
        finally:
            self.waiter.wake()
            thread.join(6)

    def test_wait_records_requested_elapsed_and_wake_reason(self):
        self.assertEqual(self.waiter.wait(.005), 'timer')
        result = self.waiter.last_wait
        self.assertAlmostEqual(result['requested_ms'], 5)
        self.assertGreaterEqual(result['elapsed_ms'], 4)
        self.assertGreaterEqual(result['overrun_ms'], 0)

    def test_close_is_idempotent_and_late_wakeup_safe(self):
        self.waiter.close()
        self.waiter.close()
        self.waiter.wake()
        self.assertEqual(self.waiter.wait(.01), 'closed')

    @unittest.skipUnless(os.name == 'nt', 'Windows socket-event integration')
    def test_queued_udp_wakes_without_consuming_packet_or_sending_anything(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(('127.0.0.1', 0))
            receiver.setblocking(False)
            try:
                self.waiter.set_socket(receiver)
                sender.sendto(b'local-only-reply', receiver.getsockname())
                self.assertEqual(self.waiter.wait(2), 'socket')
                self.assertEqual(receiver.recv(128), b'local-only-reply')
                # Readiness was reset atomically; no busy loop on an old event.
                self.assertEqual(self.waiter.wait(.003), 'timer')
                sender.sendto(b'second-reply', receiver.getsockname())
                self.assertEqual(self.waiter.wait(2), 'socket')
                self.assertEqual(receiver.recv(128), b'second-reply')
            finally:
                self.waiter.set_socket(None)

    @unittest.skipUnless(os.name == 'nt', 'Windows socket-event integration')
    def test_socket_replacement_does_not_keep_old_readiness(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as old, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as new, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            old.bind(('127.0.0.1', 0))
            new.bind(('127.0.0.1', 0))
            old.setblocking(False)
            new.setblocking(False)
            try:
                self.waiter.set_socket(old)
                sender.sendto(b'old', old.getsockname())
                self.waiter.set_socket(new)
                self.assertEqual(self.waiter.wait(.003), 'timer')
                sender.sendto(b'new', new.getsockname())
                self.assertEqual(self.waiter.wait(2), 'socket')
                self.assertEqual(new.recv(32), b'new')
            finally:
                self.waiter.set_socket(None)


if __name__ == '__main__':
    unittest.main()
