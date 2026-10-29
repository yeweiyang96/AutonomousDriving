"""North/west world-coordinate map. All interaction uses one inverse transform."""
from __future__ import annotations

from dataclasses import dataclass
from collections import OrderedDict
import math
import time
import tkinter as tk

from PIL import Image, ImageTk

from .models import Point, Route, Telemetry
from .maptiles import ImageCache, TilePyramid
from .submaps import Submaps, OverlayTile


@dataclass
class Viewport:
    c: int = 0
    north: float = -4000.0
    west: float = -1000.0
    scale: float = 0.04
    width: int = 900
    height: int = 650

    def screen(self, point: Point) -> tuple[float, float]:
        return (self.width / 2 + (self.west - point.y) * self.scale,
                self.height / 2 + (self.north - point.x) * self.scale)

    def world(self, px: float, py: float) -> Point:
        return Point(self.c, self.north - (py - self.height / 2) / self.scale,
                     self.west - (px - self.width / 2) / self.scale)

    def zoom(self, factor: float, px: float, py: float, max_scale: float = 4.0) -> None:
        anchor = self.world(px, py)
        self.scale = min(max_scale, max(0.012, self.scale * factor))
        after = self.world(px, py)
        self.north += anchor.x - after.x
        self.west += anchor.y - after.y


COLORS = {'reviewed': '#71c9a1', 'candidate': '#e5a652', 'terrain': '#d894ea', 'navmesh': '#6cbaff'}


class MapView(tk.Canvas):
    def __init__(self, parent, bundle, on_pick, on_hover=lambda p: None, **kw):
        super().__init__(parent, bg='#13232d', highlightthickness=0, **kw)
        self.bundle = bundle
        self.on_pick, self.on_hover = on_pick, on_hover
        self.view = Viewport()
        self.route: Route | None = None
        self.start: Point | None = None
        self.goal: Point | None = None
        self.pose: Telemetry | None = None
        self.track: list[Point] = []
        self.candidates = False
        self.pyramid = TilePyramid.from_bundle(bundle)
        self.submaps = Submaps.from_bundle(bundle)
        self.show_submaps = True
        self._shown_submaps = 0
        self._images = ImageCache()
        self._tile_items = {}
        self._overlay_photos = OrderedDict()
        self._overlay_photo_bytes = 0
        self._loading_tiles = False
        self._zoom_target = None
        self._zoom_job = None
        self._zoom_anchor = (0, 0)
        self._zoom_clock = 0.0
        self._drag = None
        self._redraw_job = None
        self._initial_fit = True
        self.bind('<Configure>', self._resize)
        self.bind('<MouseWheel>', self._wheel)
        self.bind('<ButtonPress-1>', self._down)
        self.bind('<B1-Motion>', self._motion)
        self.bind('<ButtonRelease-1>', self._up)
        self.bind('<Motion>', lambda e: self.on_hover(self.view.world(e.x, e.y)))
        self.bind('<Button-3>', lambda e: self.on_pick(self.view.world(e.x, e.y), True))
        self.bind('<Destroy>', self._destroy)
        self._image_job = self.after(25, self._poll_images)
        self.fit_continent(0)

    def _resize(self, event):
        self.view.width, self.view.height = max(1, event.width), max(1, event.height)
        if self._initial_fit and event.width > 100 and event.height > 100:
            self._initial_fit = False
            if self.route:
                self.fit_route()
            else:
                self.fit_continent(self.view.c)
        else:
            self.schedule_redraw()

    def schedule_redraw(self):
        if self._redraw_job is None:
            self._redraw_job = self.after(8, self.redraw)

    def _poll_images(self):
        if self._images.poll():
            self.schedule_redraw()
        self._image_job = self.after(25, self._poll_images)

    def _destroy(self, event):
        if event.widget is not self:
            return
        for job in (self._redraw_job, self._zoom_job, self._image_job):
            if job:
                self.after_cancel(job)
        self._images.close()
        self._tile_items.clear()
        self._overlay_photos.clear()
        self._overlay_photo_bytes = 0

    def _stop_zoom(self):
        if self._zoom_job:
            self.after_cancel(self._zoom_job)
        self._zoom_job, self._zoom_target = None, None

    def fit_continent(self, c: int):
        self._stop_zoom()
        points = [p for p in self.bundle.nodes if p.c == c]
        self.view.c = c
        if points:
            lo_x, hi_x = min(p.x for p in points), max(p.x for p in points)
            lo_y, hi_y = min(p.y for p in points), max(p.y for p in points)
            self.view.north, self.view.west = (lo_x + hi_x) / 2, (lo_y + hi_y) / 2
            self.view.scale = max(.012, min(self.view.width / max(hi_y - lo_y, 1),
                                             self.view.height / max(hi_x - lo_x, 1)) * .85)
        self.schedule_redraw()

    def center_player(self):
        if self.pose:
            self._stop_zoom()
            self.view.c, self.view.north, self.view.west = self.pose.c, self.pose.x, self.pose.y
            self.view.scale = max(self.view.scale, .25)
            self.schedule_redraw()

    def fit_route(self):
        if not self.route:
            return
        self._stop_zoom()
        points = self.route.points
        lo_x, hi_x = min(p.x for p in points), max(p.x for p in points)
        lo_y, hi_y = min(p.y for p in points), max(p.y for p in points)
        self.view.c = self.route.start.c
        self.view.north, self.view.west = (lo_x + hi_x) / 2, (lo_y + hi_y) / 2
        self.view.scale = min(2, max(.012, .8 * min(self.view.width / max(hi_y - lo_y, 100),
                                                   self.view.height / max(hi_x - lo_x, 100))))
        self.schedule_redraw()

    def _wheel(self, event):
        if not event.delta:
            return
        base = self._zoom_target if self._zoom_target is not None else self.view.scale
        steps = min(8, max(-8, event.delta / 120))
        limit = self.submaps.zoom_limit(self.view,self.view.world(event.x,event.y)) if self.show_submaps else 4.
        # Panning away from a small cave must not abruptly jump the zoom out.
        self._zoom_target = min(max(limit,self.view.scale), max(.012, base * 1.25 ** steps))
        self._zoom_anchor = (event.x, event.y)
        if self._zoom_job is None:
            self._zoom_clock = time.perf_counter()
            self._zoom_job = self.after(1, self._animate_zoom)

    def _animate_zoom(self):
        self._zoom_job = None
        if self._zoom_target is None:
            return
        now = time.perf_counter()
        elapsed = min(.1, max(.001, now - self._zoom_clock))
        self._zoom_clock = now
        remaining = math.log(self._zoom_target / self.view.scale)
        done = abs(remaining) < .002
        factor = self._zoom_target / self.view.scale if done else math.exp(remaining * (1 - math.exp(-20 * elapsed)))
        self.view.zoom(factor, *self._zoom_anchor, max_scale=max(4.,self.view.scale,self._zoom_target))
        self.schedule_redraw()
        if done:
            self._zoom_target = None
        else:
            self._zoom_job = self.after(16, self._animate_zoom)

    def _down(self, event):
        self._stop_zoom()
        self._drag = (event.x, event.y, self.view.north, self.view.west)

    def _motion(self, event):
        if self._drag:
            x, y, north, west = self._drag
            self.view.north = north + (event.y - y) / self.view.scale
            self.view.west = west + (event.x - x) / self.view.scale
            self.schedule_redraw()

    def _up(self, event):
        if self._drag and math.hypot(event.x - self._drag[0], event.y - self._drag[1]) < 5:
            self.on_pick(self.view.world(event.x, event.y), bool(event.state & 1))
        self._drag = None

    def set_route(self, route: Route | None):
        self.route = route
        self.draw_route()

    def _visible(self, x0, y0, x1, y1):
        return max(x0, x1) >= -20 and min(x0, x1) <= self.view.width + 20 and \
            max(y0, y1) >= -20 and min(y0, y1) <= self.view.height + 20

    def _draw_tiles(self):
        self._images.poll()
        lod = self.pyramid.choose_level(self.view.c, self.view.scale)
        primary = self.pyramid.visible(self.view, lod)
        drawn = [(tile, self._images.get(tile.path)) for tile in primary]
        self._loading_tiles = any(image is None for _, image in drawn)
        if self._loading_tiles:
            ancestors = [n for n in self.pyramid.levels.get(self.view.c, []) if n > lod]
            fallbacks = []
            for n in reversed(ancestors):
                for tile in self.pyramid.visible(self.view, n):
                    # Decode the nearest overview and the tiny continent overview first;
                    # other warm levels may also cover tiles that are still loading.
                    source = self._images.get(tile.path, request=n in (ancestors[0], ancestors[-1]))
                    if source is not None:
                        fallbacks.append((tile, source))
            drawn = fallbacks + drawn
        drawn = [(tile,source,1.) for tile,source in drawn]
        groups = {}
        if self.show_submaps:
            for tile,opacity,primary in self.submaps.visible(self.view):
                groups.setdefault(tile.area,[]).append((tile,self._images.get(tile.path),opacity,primary))
        self._shown_submaps = 0
        for group in groups.values():
            primary = [t for t in group if t[3]]
            ready = all(t[1] is not None for t in primary)
            self._loading_tiles |= not ready
            # Use one level per transparent region. Drawing fine tiles over a
            # translucent overview would blend the same artwork twice.
            selected = primary if ready else [t for t in group if not t[3] and t[1] is not None]
            if not selected:selected = [t for t in primary if t[1] is not None]
            if selected:self._shown_submaps += 1
            drawn.extend(t[:3] for t in sorted(selected,key=lambda t:(-t[0].n,t[0].row,t[0].col)))
        wanted = set()
        for tile, source, opacity in drawn:
            if source is None:
                continue
            raw_x, raw_y = self.view.screen(Point(tile.c, tile.north, tile.west))
            x0, y0 = round(raw_x), round(raw_y)
            # Stable dimensions let panning reuse PhotoImages. Rounding both
            # endpoints alternates between two sizes and resamples every frame.
            # Ceil overlaps adjacent tiles by at most one pixel, avoiding seams.
            width = height = math.ceil(tile.size * self.view.scale)
            if isinstance(tile,OverlayTile):
                # Shared rounded edges: no duplicated alpha seam at tile joins.
                width=round(raw_x+tile.size*self.view.scale)-x0
                height=round(raw_y+tile.size*self.view.scale)-y0
            if width <= 0 or height <= 0:
                continue
            left, top, right, bottom = 0, 0, width, height
            # Large overzoom is cropped to the viewport before resampling. Ordinary
            # tiles keep their image while panning: only canvas coordinates change.
            if width > 1024 or height > 1024:
                # Retain a small, grid-aligned margin while panning, so large
                # overzoom crops are reused instead of regenerated per pixel.
                step = 128
                left = max(0, ((-x0 - step) // step) * step)
                top = max(0, ((-y0 - step) // step) * step)
                right = min(width, math.ceil((self.view.width - x0 + step) / step) * step)
                bottom = min(height, math.ceil((self.view.height - y0 + step) / step) * step)
            if right <= left or bottom <= top:
                continue
            key = tile.path
            wanted.add(key)
            signature = (width, height, left, top, right, bottom, opacity)
            existing = self._tile_items.get(key)
            if existing is None or existing[2] != signature:
                cache_key=(key,signature)
                cached=self._overlay_photos.get(cache_key)
                if cached is not None:
                    photo=cached[0]
                    self._overlay_photos.move_to_end(cache_key)
                else:
                    if (left, top, right, bottom) == (0, 0, width, height):
                        pixels = source.resize((width, height), Image.Resampling.BILINEAR)
                    else:
                        extent = (left * source.width / width, top * source.height / height,
                                  right * source.width / width, bottom * source.height / height)
                        pixels = source.transform((right - left, bottom - top), Image.Transform.EXTENT,
                                                  extent, Image.Resampling.BILINEAR)
                    if opacity < 1:
                        pixels.putalpha(pixels.getchannel('A').point([round(a*opacity) for a in range(256)]))
                    photo = ImageTk.PhotoImage(pixels, master=self)
                    pixels.close()
                    if isinstance(tile,OverlayTile):
                        # Rounded shared borders alternate by one pixel during
                        # panning. Reuse those variants without redrawing alpha
                        # twice at a seam or resampling every half-pixel move.
                        size=(right-left)*(bottom-top)*4
                        self._overlay_photos[cache_key]=(photo,size)
                        self._overlay_photo_bytes+=size
                        while self._overlay_photo_bytes>32*1024*1024 and self._overlay_photos:
                            _,(_,old_size)=self._overlay_photos.popitem(last=False)
                            self._overlay_photo_bytes-=old_size
                if existing is None:
                    item = self.create_image(x0 + left, y0 + top, image=photo, anchor='nw', tags='tiles')
                else:
                    item = existing[0]
                    self.itemconfigure(item, image=photo)
                self._tile_items[key] = (item, photo, signature)
            item = self._tile_items[key][0]
            self.coords(item, x0 + left, y0 + top)
            self.tag_raise(item)
        for key in list(self._tile_items):
            if key not in wanted:
                self.delete(self._tile_items.pop(key)[0])
        self.tag_lower('tiles')

    def redraw(self):
        if self._redraw_job:
            self.after_cancel(self._redraw_job)
        self._redraw_job = None
        if not self.winfo_exists():
            return
        self.delete('roads', 'legend')
        self._draw_tiles()
        for edge in self.bundle.edges:
            if edge.kind != 'walk':
                continue
            reviewed = edge.status in ('user_reviewed', 'user_correction')
            if not reviewed and not self.candidates:
                continue
            a, b = self.bundle.nodes[edge.a], self.bundle.nodes[edge.b]
            if a.c != self.view.c:
                continue
            ax, ay = self.view.screen(a)
            bx, by = self.view.screen(b)
            if self._visible(ax, ay, bx, by):
                self.create_line(ax, ay, bx, by, fill='#3e7e6a' if reviewed else '#87633a',
                                 width=1, tags='roads')
        self.draw_route()
        self.draw_pose()
        self.create_rectangle(12, 12, 462, 44, fill='#14232d', outline='', tags='legend')
        for i, (label, reliability) in enumerate((('已审核道路', 'reviewed'), ('候选道路', 'candidate'), ('地形估算', 'terrain'), ('碰撞导航', 'navmesh'))):
            self.create_text(24 + i * 112, 28, text='● ' + label,
                             fill=COLORS[reliability], anchor='w', font=('Microsoft YaHei UI', 10), tags='legend')
        label = '高清地形 · 本地离线' if self.pyramid.build else '概览地图'
        if self._shown_submaps:
            label = '高清地形 + 洞穴 / 主城'
        if self.pyramid.error or self.submaps.error or self._images.failed:
            label += ' · 部分底图不可用'
        elif self._loading_tiles:
            label += ' · 加载中'
        self.create_rectangle(12, self.view.height - 36, 272, self.view.height - 10,
                              fill='#14232d', outline='', tags='legend')
        self.create_text(24, self.view.height - 23, text=label, fill='#c2d1d9', anchor='w',
                         font=('Microsoft YaHei UI', 10), tags='legend')

    def draw_route(self):
        self.delete('route')
        if self.route:
            points, style = [], None
            def flush():
                if len(points) >= 4:
                    self.create_line(*points, fill=COLORS.get(style, '#fff'), width=3,
                                     dash=(6, 3) if style == 'terrain' else (), tags='route')
            for part in self.route.segments:
                if part.points and part.points[0].c == self.view.c and len(part.points) >= 2:
                    following = [v for p in part.points for v in self.view.screen(p)]
                    if style == part.reliability and points[-2:] == following[:2]:
                        points.extend(following[2:])
                    else:
                        flush()
                        points, style = following, part.reliability
            flush()
        for point, label, color in ((self.start, 'A', '#63c7e6'), (self.goal, 'B', '#ffa95b')):
            if point and point.c == self.view.c:
                x, y = self.view.screen(point)
                self.create_oval(x - 10, y - 10, x + 10, y + 10, fill=color, outline='#fff', tags='route')
                self.create_text(x, y, text=label, fill='#16212a', tags='route')
        self.tag_raise('pose')

    def update_pose(self, pose: Telemetry, *, draw=True):
        if self.pose and self.pose.session != pose.session:
            self.track.clear()
        self.pose = pose
        if not self.track or self.track[-1].distance(pose.point) > 1:
            self.track.append(pose.point)
            self.track = self.track[-1500:]
        if draw:
            self.draw_pose()

    def draw_pose(self):
        self.delete('pose')
        # A continuous trail is one Canvas polyline, not up to 1,499 separate
        # Tcl calls per telemetry frame. Preserve breaks at teleports/maps.
        line = []
        for a, b in zip(self.track, self.track[1:]):
            if a.c == b.c == self.view.c and a.distance(b) < 100:
                if not line:
                    line.extend(self.view.screen(a))
                line.extend(self.view.screen(b))
            elif line:
                self.create_line(*line, fill='#63c8f0', width=2, tags='pose')
                line = []
        if line:
            self.create_line(*line, fill='#63c8f0', width=2, tags='pose')
        if self.pose and self.pose.c == self.view.c:
            x, y = self.view.screen(self.pose.point)
            theta = self.pose.facing
            tip = (x - 14 * math.sin(theta), y - 14 * math.cos(theta))
            left = (x - 8 * math.sin(theta + 2.5), y - 8 * math.cos(theta + 2.5))
            right = (x - 8 * math.sin(theta - 2.5), y - 8 * math.cos(theta - 2.5))
            self.create_polygon(*tip, *left, *right, fill='#75e5ff', outline='#fff', width=1, tags='pose')
