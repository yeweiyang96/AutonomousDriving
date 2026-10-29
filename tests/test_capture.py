import unittest
from unittest.mock import patch

from PIL import Image

from adrive.capture import CaptureError, WindowsCapture
from adrive.telemetry import render_frame
from tests.test_telemetry import sample


class FakeBackend:
    def __init__(self):
        self.origin = (120, 80)
        self.image = Image.new("RGB", (800, 600), (20, 40, 80))
        self.image.paste(render_frame(sample()), (8, 8))
        self.handles = [(33, "World of Warcraft")]
        self.active = 33
        self.minimized = False
        self.grabs = []

    def windows(self):
        return self.handles

    def valid(self, hwnd):
        return any(item[0] == hwnd for item in self.handles)

    def foreground(self):
        return self.active

    def bounds(self, hwnd):
        if self.minimized:
            return None
        x, y = self.origin
        return x, y, x + self.image.width, y + self.image.height

    def grab(self, box):
        self.grabs.append(box)
        x, y = self.origin
        return self.image.crop((box[0] - x, box[1] - y, box[2] - x, box[3] - y))


class CaptureTests(unittest.TestCase):
    def test_binds_exact_title_then_only_captures_panel_and_tracks_move(self):
        backend = FakeBackend()
        capture = WindowsCapture(_backend=backend)
        self.assertFalse(capture.foreground())
        first = capture.poll()
        self.assertEqual(sample().point, first.point)
        self.assertTrue(capture.foreground())
        second = capture.poll()
        self.assertEqual(first.seq, second.seq)  # FreshnessTracker rejects duplicates.
        self.assertEqual((128, 88, 256, 152), backend.grabs[-1])
        backend.origin = (-1000, 30)  # Monitor left of the primary screen.
        self.assertEqual(first.point, capture.poll().point)
        self.assertEqual((-992, 38, -864, 102), backend.grabs[-1])

    def test_does_not_bind_browser_or_ambiguous_game_title(self):
        backend = FakeBackend()
        backend.handles = [(66, "World of Warcraft - Microsoft Edge")]
        capture = WindowsCapture(_backend=backend)
        self.assertIsNone(capture.poll())
        self.assertFalse(capture.foreground())
        backend.handles = [(33, "World of Warcraft"), (34, "World of Warcraft")]
        self.assertIsNone(capture.poll())
        self.assertIn("多个", capture.status)
        selected = WindowsCapture(hwnd=34, _backend=backend)
        selected.poll()
        self.assertFalse(selected.foreground())

    def test_minimize_occlusion_and_closed_capture_never_reuse_frame(self):
        backend = FakeBackend()
        capture = WindowsCapture(_backend=backend)
        self.assertIsNotNone(capture.poll())
        backend.image.paste((0, 0, 0), (8, 8, 136, 72))
        self.assertIsNone(capture.poll())
        backend.minimized = True
        self.assertIsNone(capture.poll())
        self.assertIn("最小化", capture.status)
        capture.close()
        self.assertIsNone(capture.poll())
        self.assertFalse(capture.foreground())

    def test_resizing_relocates_scaled_panel(self):
        backend = FakeBackend()
        capture = WindowsCapture(_backend=backend)
        self.assertIsNotNone(capture.poll())
        backend.image = Image.new("RGB", (900, 700))
        backend.image.paste(render_frame(sample(), 12), (30, 15))
        with patch("adrive.capture.time.monotonic", return_value=100000000):
            frame = capture.poll()
        self.assertIsNotNone(frame)
        self.assertEqual(sample().point, frame.point)

    def test_capture_error_is_visible_and_no_frame_is_fabricated(self):
        backend = FakeBackend()
        capture = WindowsCapture(_backend=backend)
        backend.grab = lambda box: (_ for _ in ()).throw(CaptureError("test failure"))
        self.assertIsNone(capture.poll())
        self.assertEqual("test failure", capture.status)

    def test_recycled_window_handle_does_not_authorize_input_in_other_app(self):
        backend = FakeBackend()
        capture = WindowsCapture(_backend=backend)
        self.assertIsNotNone(capture.poll())
        backend.handles = [(33, "Text Editor")]
        self.assertFalse(capture.foreground())
        self.assertIsNone(capture.poll())
        self.assertIsNone(capture.hwnd)


if __name__ == "__main__":
    unittest.main()
