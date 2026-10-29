"""Optional Windows screen-capture smoke test using this project's own window.

Draws a short-lived, explicitly named Tk fixture. It never accesses another
application's controls, sends keyboard/mouse input, or connects to a Pico.
Run from the repository root: .venv/Scripts/python.exe tools/check_capture.py
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from adrive.capture import WindowsCapture, enable_dpi_awareness
from adrive.models import Flags, Telemetry
from adrive.telemetry import FreshnessTracker, render_frame


TITLE = "AutonomousDriving Capture Fixture"
SESSION = 0x5A114A47


def run(seconds: float = 6.0, cell_size: int = 12) -> dict:
    result = {
        "ok": False,
        "test": "Win32 BitBlt capture of own Tk telemetry fixture",
        "fixture_title": TITLE,
        "duration_requested_seconds": seconds,
        "cell_size_pixels": cell_size,
        "keyboard_mouse_actions": 0,
        "pico_connections": 0,
        "rendered_frames": 0,
        "decoded_frames": 0,
        "new_frames": 0,
        "duplicate_or_old_frames": 0,
        "content_mismatches": 0,
        "reason": "",
    }
    if os.name != "nt":
        result["reason"] = "实时 Win32 烟测仅支持 Windows"
        return result
    root = capture = None
    began = time.monotonic()
    unique_times: list[float] = []
    sequences: list[int] = []
    tracker = FreshnessTracker()
    errors: list[str] = []
    try:
        import tkinter as tk
        from PIL import ImageTk

        enable_dpi_awareness()
        root = tk.Tk()
        root.title(TITLE)
        root.resizable(False, False)
        root.configure(background="#202020")
        root.geometry("+80+80")
        # Applies only to our own fixture; no other window is activated/moved.
        root.attributes("-topmost", True)
        image_label = tk.Label(root, borderwidth=0, highlightthickness=0, background="#202020")
        image_label.pack(padx=8, pady=8)
        message = tk.Label(root, text=f"自主测试窗口 · 约 {seconds:g} 秒后关闭", foreground="white",
                           background="#202020", font=("Microsoft YaHei UI", 9))
        message.pack(padx=8, pady=(0, 8))
        capture = WindowsCapture(window_title=TITLE)
        sequence = 0
        done = False
        photo = None
        draw_deadline = time.monotonic()
        observe_deadline = draw_deadline + 0.120

        def finish(user_closed=False):
            nonlocal done
            if done:
                return
            done = True
            if user_closed:
                errors.append("测试窗口被提前关闭")
            root.quit()

        root.protocol("WM_DELETE_WINDOW", lambda: finish(True))

        def callback_error(exc_type, value, traceback):
            errors.append(f"{exc_type.__name__}: {value}")
            finish()

        root.report_callback_exception = callback_error

        def draw():
            nonlocal sequence, photo, draw_deadline
            if done:
                return
            sequence += 1
            telemetry = Telemetry(SESSION, sequence, 1, 1411, -1234.5, 5678.25,
                                  math.pi / 2, 7, Flags.VALID)
            photo = ImageTk.PhotoImage(render_frame(telemetry, cell_size))
            image_label.configure(image=photo)
            result["rendered_frames"] = sequence
            # Absolute deadlines avoid accumulating Windows timer rounding and
            # drawing time into a nominal 33 ms interval.
            draw_deadline += 1 / 30
            root.after(max(1, math.ceil((draw_deadline - time.monotonic()) * 1000)), draw)

        def observe():
            nonlocal observe_deadline
            if done:
                return
            telemetry = capture.poll()
            now = time.monotonic()
            if telemetry is not None:
                result["decoded_frames"] += 1
                correct = (
                    telemetry.session == SESSION and 0 < telemetry.seq <= sequence
                    and telemetry.c == 1 and telemetry.ui_map == 1411
                    and telemetry.x == -1234.5 and telemetry.y == 5678.25
                    and abs(telemetry.facing - math.pi / 2) < 1e-12
                    and telemetry.speed == 7 and telemetry.flags == Flags.VALID
                )
                if not correct:
                    result["content_mismatches"] += 1
                elif tracker.accept(telemetry, now):
                    unique_times.append(now)
                    sequences.append(telemetry.seq)
                    result["new_frames"] += 1
                else:
                    result["duplicate_or_old_frames"] += 1
            observe_deadline += 1 / 30
            root.after(max(1, math.ceil((observe_deadline - time.monotonic()) * 1000)), observe)

        draw()
        root.after(120, observe)  # Let Tk and the Windows compositor present it.
        root.after(round(seconds * 1000), finish)
        root.mainloop()
        result["window_handle"] = capture.hwnd
        result["last_capture_status"] = capture.status
    except Exception as error:
        errors.append(f"{type(error).__name__}: {error}")
    finally:
        if capture is not None:
            capture.close()
        if root is not None:
            try:
                root.destroy()
            except Exception as error:
                errors.append(f"关闭测试窗口失败: {error}")
        result["elapsed_seconds"] = round(time.monotonic() - began, 3)

    result["first_frame_after_ms"] = round((unique_times[0] - began) * 1000, 1) if unique_times else None
    result["observed_new_frame_hz"] = round((len(unique_times) - 1) / (unique_times[-1] - unique_times[0]), 2) if len(unique_times) >= 2 else 0
    result["max_new_frame_gap_ms"] = round(max((b - a for a, b in zip(unique_times, unique_times[1:])), default=0) * 1000, 1)
    result["first_sequence"] = sequences[0] if sequences else None
    result["last_sequence"] = sequences[-1] if sequences else None
    result["errors"] = errors
    result["ok"] = not errors and result["content_mismatches"] == 0 and result["new_frames"] >= 10
    if result["ok"]:
        result["reason"] = "自有窗口经真实 Win32 屏幕抓取、定位和解码取得至少 10 个一致的新帧"
    elif errors:
        result["reason"] = "测试异常，未通过"
    elif result["content_mismatches"]:
        result["reason"] = "捕获内容与本测试窗口不一致，未通过"
    else:
        result["reason"] = "有效新帧不足，未通过；检查测试窗口是否被遮挡、远程会话或屏幕色彩设置"
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--cell-size", type=int, default=12)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 8 or not 4 <= args.cell_size <= 24:
        parser.error("--seconds 必须为 5–8，--cell-size 必须为 4–24")
    result = run(args.seconds, args.cell_size)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
