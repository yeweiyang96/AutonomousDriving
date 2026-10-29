"""Measure offline Tk map redraws (not display refresh or live game performance)."""
from __future__ import annotations

import json
import argparse
from pathlib import Path
import statistics
import sys
import time
import tkinter as tk

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from adrive.data import DataBundle
from adrive.mapview import MapView


def run(submaps=False):
    root = tk.Tk()
    root.withdraw()
    view = MapView(root, DataBundle.load(ROOT / 'data'), lambda *_: None)
    view._initial_fit = False
    results = []
    try:
        scenes=[(0,-9458.437,50.658,(.04,.25,.96,4.)),(1,1630.,-4400.,(.04,.25,.96,4.))]
        if submaps:
            scenes=[(0,-4850.,-1138.,(.2,.95,2.)),(1,1490.,-4764.,(2.3,)),(0,3070.,-1540.,(8.,))]
        for c, north, west, scales in scenes:
            for scale in scales:
                view.view.c, view.view.north, view.view.west = c, north, west
                view.view.scale, view.view.width, view.view.height = scale, 1100, 720
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    root.update()
                    view.redraw()
                    if not view._images.pending and not view._loading_tiles:
                        break
                    time.sleep(.01)
                times = []
                for i in range(60):
                    view.view.west = west + i * .5 / scale
                    start = time.perf_counter()
                    view.redraw()
                    times.append((time.perf_counter() - start) * 1000)
                results.append({'continent': c, 'pixels_per_yard': scale,
                                'median_ms': round(statistics.median(times), 2),
                                'p95_ms': round(sorted(times)[56], 2),
                                'visible_tile_images': len(view._tile_items),
                                'source_cache_mib': round(view._images.bytes / 1048576, 1),
                                'submap_photo_cache_mib': round(view._overlay_photo_bytes / 1048576,1),
                                'visible_submaps': view._shown_submaps,
                                'decode_failures': len(view._images.failed)})
        return {'scope': 'Offline Tk redraw only; no game or device connection; 1100x720 viewport',
                'build': view.pyramid.build, 'results': results}
    finally:
        root.destroy()


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--submaps',action='store_true',help='检查铁炉堡、洞穴与深度缩放')
    args=parser.parse_args()
    result = run(args.submaps)
    path = ROOT / ('runtime/submap-benchmark.json' if args.submaps else 'runtime/map-benchmark.json')
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
