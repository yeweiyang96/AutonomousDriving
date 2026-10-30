"""Offline road-layer benchmark; never captures/controls the game or a device."""
import argparse
import importlib.util
import json
from pathlib import Path
import statistics
import sys
import tempfile
import time
import tkinter as tk
from dataclasses import replace
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from adrive.data import DataBundle
from adrive.mapview import MapView
from adrive.road_display import RoadDisplaySource


def run(legacy=False,display=None):
    cls=MapView
    if legacy:
        spec=importlib.util.spec_from_file_location('adrive._mapview_before',ROOT/'runtime/road-display-performance/mapview_before.py')
        module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module);cls=module.MapView
    root=tk.Tk();root.withdraw();bundle=DataBundle.load(ROOT/'data');proxy=None
    if display:
        import hashlib,shutil
        proxy=tempfile.TemporaryDirectory(prefix='road-artwork-preview-',dir=ROOT/'runtime/road-display-performance')
        directory=Path(proxy.name)/'roads';directory.mkdir();data=display.read_bytes();shutil.copyfile(display,directory/'display.json')
        (directory/'manifest.json').write_text(json.dumps(dict(schema=1,kind='adrive-road-pointer',path='unused-execution-manifest.json',
            display=dict(path='display.json',bytes=len(data),sha256=hashlib.sha256(data).hexdigest()))))
        original=RoadDisplaySource;factory=lambda b:original(replace(b,asset_root=Path(proxy.name)))
    else:factory=RoadDisplaySource
    started=time.perf_counter()
    with patch('adrive.mapview.RoadDisplaySource',factory):view=cls(root,bundle,lambda *_:None)
    initial=(time.perf_counter()-started)*1000
    view._initial_fit=False;view._draw_tiles=lambda:None
    if view._redraw_job:view.after_cancel(view._redraw_job);view._redraw_job=None
    rows=[]
    try:
        if not legacy:
            deadline=time.monotonic()+20
            while time.monotonic()<deadline and view._roads.loading:
                root.update();time.sleep(.01)
        scenes=[('continent',0,-5500.,-1000.,.04),('dun_morogh',0,-5500.,-2000.,.3),('ironforge',0,-4850.,-1138.,.95)]
        for name,c,north,west,scale in scenes:
            view.view.c,view.view.north,view.view.west=c,north,west
            view.view.scale,view.view.width,view.view.height=scale,943,557
            began=time.perf_counter();view.redraw()
            if not legacy:
                deadline=time.monotonic()+20
                while time.monotonic()<deadline and (view._roads.pending or view._road_paint_job
                        or view._roads.frame is None or view._road_frame is not view._roads.frame):
                    root.update();time.sleep(.001)
            first=(time.perf_counter()-began)*1000
            samples=[]
            for i in range(12):
                view.view.west=west+i*.5/scale
                t=time.perf_counter();view.redraw();samples.append((time.perf_counter()-t)*1000)
            pan_items=len(view.find_withtag('roads'));zoom=[]
            for i in range(12):
                view.view.scale=scale*(1.02**i)
                t=time.perf_counter();view.redraw();zoom.append((time.perf_counter()-t)*1000)
            rows.append(dict(name=name,pixels_per_yard=scale,initial_layer_ms=round(first,3),
                warm_pan_median_ms=round(statistics.median(samples),3),warm_pan_max_ms=round(max(samples),3),
                warm_zoom_median_ms=round(statistics.median(zoom),3),warm_zoom_max_ms=round(max(zoom),3),
                canvas_road_items=pan_items,canvas_items_after_zoom=len(view.find_withtag('roads'))))
        return dict(scope='Offline Tk road layer only; no base tiles, telemetry, game or device input',
            legacy=legacy,display=str(display) if display else None,
            display_origin=view._roads.origin if not legacy else 'fragmented_execution_graph',
            display_error=view._roads.error if not legacy else '',
            source_lines=view._roads.line_count if not legacy else None,
            constructor_ms=round(initial,3),results=rows)
    finally:
        root.destroy()
        if proxy:proxy.cleanup()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--legacy',action='store_true')
    parser.add_argument('--display',type=Path,help='Preview candidate artwork without publishing its execution graph')
    parser.add_argument('--report',type=Path)
    args=parser.parse_args();result=run(args.legacy,args.display)
    output=args.report or ROOT/'runtime/road-display-performance'/('before.json' if args.legacy else 'after.json')
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))
