from dataclasses import replace
import math
import unittest

from PIL import Image, ImageDraw

from adrive.models import Flags, Telemetry, angle_error
from adrive.telemetry import (
    DecodeError, FRAME_BYTES, FreshnessTracker, decode_frame, decode_image,
    encode_frame, locate_panel, render_frame,
)


def sample(**changes):
    return replace(Telemetry(0x12345678, 0xFFFFFFFE, 1, 1411, -1234.5, 5678.25,
                             math.pi / 2, 7, Flags.VALID), **changes)


class BinaryProtocolTests(unittest.TestCase):
    def test_golden_packet_and_endianness(self):
        # Also recorded in docs/telemetry-protocol.md for Lua verification.
        expected = bytes.fromhex("4144010078563412feffffff010083050000d8b2ffffe4620100004000070100066c")
        self.assertEqual(34, FRAME_BYTES)
        self.assertEqual(expected, encode_frame(sample()))
        self.assertEqual(sample(captured_at=12), decode_frame(expected, 12))

    def test_signed_coordinates_precision_facing_wrap_and_flags(self):
        for x in (-17000.123, -0.03125, 0.03125, 17000.987):
            for facing in (-0.001, math.tau - 0.000001, 15 * math.tau + 0.5):
                t = sample(x=x, y=-x, facing=facing, flags=Flags.COMBAT | Flags.LOADING)
                result = decode_frame(encode_frame(t), 1)
                self.assertLessEqual(abs(t.x - result.x), 1 / 32)
                self.assertLessEqual(abs(t.y - result.y), 1 / 32)
                self.assertLessEqual(abs(angle_error(t.facing, result.facing)), math.pi / 65536)
                self.assertEqual(t.flags, result.flags)

    def test_corruption_and_truncation_fail_closed(self):
        raw = encode_frame(sample())
        for index in range(len(raw)):
            bad = bytearray(raw)
            bad[index] ^= 0x01
            with self.assertRaises(DecodeError):
                decode_frame(bytes(bad))
        with self.assertRaises(DecodeError):
            decode_frame(raw[:-1])

    def test_invalid_numbers_rejected(self):
        for change in ({"x": math.nan}, {"facing": math.inf}, {"speed": -1}, {"flags": Flags(1024)}):
            with self.assertRaises(ValueError):
                encode_frame(sample(**change))


class PixelProtocolTests(unittest.TestCase):
    def test_full_frame_inside_offset_screenshot(self):
        image = Image.new("RGB", (750, 400), (40, 60, 80))
        image.paste(render_frame(sample()), (59, 17))
        panel = locate_panel(image)
        self.assertEqual((59, 17, 8), (panel.x, panel.y, panel.cell))
        self.assertEqual(sample(captured_at=4), decode_image(image, 4))

    def test_display_scaling_and_noninteger_ui_scale(self):
        for scale in (0.5, 0.75, 0.9, 1, 1.25, 1.333, 1.5, 2, 2.5):
            with self.subTest(scale=scale):
                frame = render_frame(sample())
                frame = frame.resize((round(frame.width * scale), round(frame.height * scale)), Image.Resampling.BILINEAR)
                image = Image.new("RGB", (500, 250), (20, 30, 40))
                image.paste(frame, (23, 11))
                self.assertEqual(sample(captured_at=4), decode_image(image, 4))

    def test_calibration_handles_gamma_shift(self):
        gamma = [round(255 * (index / 255) ** 0.65) for index in range(256)]
        image = render_frame(sample()).point(gamma * 3)
        self.assertEqual(sample(captured_at=4), decode_image(image, 4))

    def test_bad_data_marker_and_palette_are_rejected(self):
        for box in ((0, 16, 7, 23), (0, 0, 7, 7), (0, 8, 127, 15), (32, 48, 39, 55)):
            image = render_frame(sample())
            ImageDraw.Draw(image).rectangle(box, fill=(128, 64, 24))
            with self.assertRaises(DecodeError):
                decode_image(image)


class FreshnessTests(unittest.TestCase):
    def test_duplicates_out_of_order_and_freeze_do_not_refresh(self):
        tracker = FreshnessTracker()
        self.assertTrue(tracker.accept(sample(seq=20), 10))
        self.assertFalse(tracker.accept(sample(seq=20), 10.2))
        self.assertFalse(tracker.accept(sample(seq=19), 10.23))
        self.assertTrue(tracker.fresh(10.25))
        self.assertFalse(tracker.fresh(10.251))

    def test_sequence_wrap_and_retired_sessions(self):
        tracker = FreshnessTracker()
        self.assertTrue(tracker.accept(sample(seq=0xFFFFFFFF), 10))
        self.assertTrue(tracker.accept(sample(seq=0), 10.1))
        self.assertFalse(tracker.accept(sample(seq=0xFFFFFFFE), 10.15))
        self.assertTrue(tracker.accept(sample(session=99, seq=0), 11))
        self.assertFalse(tracker.accept(sample(seq=1), 11.1))
        self.assertEqual(99, tracker.latest.session)

    def test_delayed_decode_does_not_renew_old_acquisition(self):
        tracker = FreshnessTracker()
        self.assertTrue(tracker.accept(sample(captured_at=10), 10.5))
        self.assertFalse(tracker.fresh(10.5))
        self.assertFalse(tracker.accept(sample(seq=1, session=33, captured_at=12), 11))
        self.assertTrue(tracker.accept(sample(seq=0xFFFFFFFF, captured_at=11), 11))

    def test_fresh_invalid_frame_does_not_reuse_old_valid_position(self):
        tracker = FreshnessTracker()
        tracker.accept(sample(seq=5), 10)
        tracker.accept(sample(seq=6, x=0, y=0, flags=Flags.LOADING), 10.1)
        self.assertTrue(tracker.fresh(10.2))
        self.assertEqual(Flags.LOADING, tracker.latest.flags)
        self.assertEqual(0, tracker.latest.x)


if __name__ == "__main__":
    unittest.main()
