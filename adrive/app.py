"""Chinese desktop map, explicit controls, and offline replay viewer."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .capture import enable_dpi_awareness, list_game_windows
from .data import DataBundle
from .hotkeys import Hotkeys
from .mapview import MapView
from .models import Flags, Point, Telemetry
from .planning_worker import IsolatedPlanner as Planner
from .replay import Replay, route_from_dict, telemetry_from_dict
from .settings import ROOT, Settings


STATE_NAMES = {'monitoring': '监测中', 'armed': '等待游戏', 'planning': '规划中',
               'running': '行走中', 'calibration_armed': '等待校准', 'calibrating': '校准中',
               'paused': '已暂停', 'stopped': '已停止', 'arrived': '已到达', 'fault': '需要处理'}
ACTIVE_STATES = {'armed', 'planning', 'running', 'calibration_armed', 'calibrating'}


def point_text(p):
    if not p:
        return '尚未选择'
    return f'{"东部王国" if p.c == 0 else "卡利姆多"}  北 {p.x:.1f} / 西 {p.y:.1f}' + (f' / 高 {p.z:.1f}' if p.z is not None else '')


class Application:
    def __init__(self, root: tk.Tk, bundle: DataBundle, *, demo=False, replay_path=None):
        self.root, self.bundle, self.demo = root, bundle, demo or bool(replay_path)
        self.events = queue.Queue()
        self.runtime = None
        self.hotkeys = None
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='preview')
        self.preview_cancel = threading.Event()
        self.preview_generation = 0
        self.preview_future = None
        self.latest = None
        self.preview_start = None
        self.replay_rows = []
        self.replay_index = 0
        self.replay_job = None
        self.state = 'monitoring'
        self.closed = False
        try:
            self.settings = Settings.load()
        except (OSError, ValueError, TypeError) as exc:
            self.settings = Settings()
            self.events.put({'kind': 'message', 'text': f'配置读取失败，已使用默认值：{exc}'})
        root.title('AutonomousDriving · Forever 地面导航' + (' · 离线预览' if self.demo else ''))
        root.geometry('1280x820')
        root.minsize(1000, 680)
        root.configure(bg='#edf2f5')
        self._build()
        root.protocol('WM_DELETE_WINDOW', self.close)
        if not self.demo:
            from .runtime import Runtime
            self.runtime = Runtime(bundle, self.settings, self.events)
            self.runtime.start()
            self.hotkeys = Hotkeys(lambda cmd: self.runtime.command(cmd),
                                   lambda text: self.events.put({'kind': 'message', 'text': text}))
            self.hotkeys.start()
        else:
            self.log('离线模式：地图选点、规划和日志回放可用，不连接 Pico，不发送输入。')
        if replay_path:
            self.load_replay(replay_path)
        root.after(50, self.poll)

    def _build(self):
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('.', font=('Microsoft YaHei UI', 10))
        style.configure('TFrame', background='#edf2f5')
        style.configure('TLabel', background='#edf2f5', foreground='#223642')
        style.configure('Title.TLabel', font=('Microsoft YaHei UI', 18, 'bold'))
        style.configure('Hint.TLabel', foreground='#586e7b', font=('Microsoft YaHei UI', 9))
        style.configure('Accent.TButton', background='#196f86', foreground='white', padding=(12, 8))
        style.configure('TButton', padding=(8, 5))
        top = ttk.Frame(self.root, padding=(18, 12))
        top.pack(fill='x')
        ttk.Label(top, text='AutonomousDriving', style='Title.TLabel').pack(side='left')
        ttk.Label(top, text='  FOREVER / 地面导航', style='Hint.TLabel').pack(side='left', padx=12)
        self.state_var = tk.StringVar(value='离线预览' if self.demo else '监测中')
        ttk.Label(top, textvariable=self.state_var, foreground='#196f86').pack(side='right')
        body = ttk.Frame(self.root, padding=(12, 0, 12, 10))
        body.pack(fill='both', expand=True)
        sidebar_outer = ttk.Frame(body, width=320, padding=(8, 0, 14, 0))
        sidebar_outer.pack(side='left', fill='y')
        sidebar_outer.pack_propagate(False)
        action_panel = ttk.Frame(sidebar_outer, padding=(0, 8, 0, 0))
        action_panel.pack(side='bottom', fill='x')
        scroll_area = ttk.Frame(sidebar_outer)
        scroll_area.pack(fill='both', expand=True)
        sidebar_canvas = tk.Canvas(scroll_area, bg='#edf2f5', highlightthickness=0, width=280)
        scrollbar = ttk.Scrollbar(scroll_area, orient='vertical', command=sidebar_canvas.yview)
        scrollbar.pack(side='right', fill='y')
        sidebar_canvas.pack(side='left', fill='both', expand=True)
        sidebar_canvas.configure(yscrollcommand=scrollbar.set)
        sidebar = ttk.Frame(sidebar_canvas)
        sidebar_window = sidebar_canvas.create_window(0, 0, window=sidebar, anchor='nw')
        sidebar.bind('<Configure>', lambda e: sidebar_canvas.configure(scrollregion=sidebar_canvas.bbox('all')))
        sidebar_canvas.bind('<Configure>', lambda e: sidebar_canvas.itemconfigure(sidebar_window, width=e.width))
        map_frame = ttk.Frame(body)
        map_frame.pack(side='left', fill='both', expand=True)
        toolbar = ttk.Frame(map_frame, padding=(0, 0, 0, 8))
        toolbar.pack(fill='x')
        self.continent = tk.StringVar(value='东部王国')
        continent = ttk.Combobox(toolbar, textvariable=self.continent, values=['东部王国', '卡利姆多'],
                                 width=12, state='readonly')
        continent.pack(side='left')
        continent.bind('<<ComboboxSelected>>', lambda _: self.map.fit_continent(0 if self.continent.get() == '东部王国' else 1))
        ttk.Button(toolbar, text='定位角色', command=lambda: self.map.center_player()).pack(side='left', padx=6)
        ttk.Button(toolbar, text='查看全程', command=lambda: self.map.fit_route()).pack(side='left')
        self.submaps_var = tk.BooleanVar(value=True)
        def toggle_submaps():
            self.map.show_submaps=self.submaps_var.get()
            self.map.schedule_redraw()
        ttk.Checkbutton(toolbar,text='洞穴 / 主城',variable=self.submaps_var,
                        command=toggle_submaps).pack(side='left',padx=8)
        ttk.Button(toolbar, text='导入回放', command=self.choose_replay).pack(side='right')
        self.map = MapView(map_frame, self.bundle, self.pick, self.hover)
        self.map.pack(fill='both', expand=True)
        self.hover_var = tk.StringVar(value='滚轮缩放 · 拖动地图 · 单击设终点 B · Shift+单击或右键设预览起点 A')
        ttk.Label(map_frame, textvariable=self.hover_var, style='Hint.TLabel').pack(anchor='w', pady=6)
        ttk.Label(sidebar, text='路线', font=('Microsoft YaHei UI', 12, 'bold')).pack(anchor='w', pady=(0, 8))
        self.start_var = tk.StringVar(value='A  使用实时位置')
        self.goal_var = tk.StringVar(value='B  尚未选择')
        ttk.Label(sidebar, textvariable=self.start_var, wraplength=275).pack(anchor='w', pady=3)
        ttk.Label(sidebar, textvariable=self.goal_var, wraplength=275).pack(anchor='w', pady=3)
        self.height_vars, self.height_boxes, self.height_values = {}, {}, {}
        for name in ('A', 'B'):
            line = ttk.Frame(sidebar)
            line.pack(fill='x', pady=2)
            ttk.Label(line, text=f'{name} 地面高度').pack(side='left')
            value = tk.StringVar(value='自动（单层地面）')
            box = ttk.Combobox(line, textvariable=value, values=['自动（单层地面）'], state='readonly', width=17)
            box.pack(side='right')
            box.bind('<<ComboboxSelected>>', lambda _, n=name: self.select_height(n))
            self.height_vars[name], self.height_boxes[name], self.height_values[name] = value, box, {}
        ttk.Button(sidebar, text='起点恢复为实时位置', command=self.clear_start).pack(fill='x', pady=5)
        self.candidates_var = tk.BooleanVar(value=self.settings.allow_candidates)
        ttk.Checkbutton(sidebar, text='允许候选地面道路', variable=self.candidates_var,
                        command=self.change_policy).pack(anchor='w', pady=4)
        ttk.Label(sidebar, text='蓝色实线：静态碰撞导航\n紫色虚线：旧地形估算\n多层位置需选择高度；实际行走仍需验证',
                  style='Hint.TLabel', wraplength=275).pack(anchor='w', pady=(0, 8))
        ttk.Button(sidebar, text='预览路线', command=self.preview).pack(fill='x')
        self.route_var = tk.StringVar(value='选择目的地后预览距离与路线来源')
        ttk.Label(sidebar, textvariable=self.route_var, wraplength=275).pack(anchor='w', pady=8)
        ttk.Separator(sidebar).pack(fill='x', pady=8)
        ttk.Label(sidebar, text='连接与按键', font=('Microsoft YaHei UI', 12, 'bold')).pack(anchor='w', pady=(0, 7))
        address = ttk.Frame(sidebar)
        address.pack(fill='x')
        self.host_var, self.port_var = tk.StringVar(value=self.settings.host), tk.StringVar(value=str(self.settings.port))
        ttk.Entry(address, textvariable=self.host_var, width=18).pack(side='left', fill='x', expand=True)
        ttk.Entry(address, textvariable=self.port_var, width=6).pack(side='left', padx=(5, 0))
        self.window_var = tk.StringVar(value=self.settings.window_title)
        self.window_combo = ttk.Combobox(sidebar, textvariable=self.window_var)
        self.window_combo.pack(fill='x', pady=6)
        refresh = ttk.Frame(sidebar)
        refresh.pack(fill='x')
        ttk.Button(refresh, text='刷新游戏窗口', command=self.refresh_windows).pack(side='left')
        ttk.Button(refresh, text='连接诊断', command=self.diagnose).pack(side='right')
        self.device_diagnostic_var = tk.StringVar(value='尚未读取固件版本与能力')
        ttk.Label(sidebar, textvariable=self.device_diagnostic_var, style='Hint.TLabel',
                  wraplength=275).pack(anchor='w', pady=(6, 0))
        keys = ttk.Frame(sidebar)
        keys.pack(fill='x', pady=8)
        self.key_vars = []
        for text, value in [('前进', self.settings.forward_key), ('左转', self.settings.left_key), ('右转', self.settings.right_key)]:
            ttk.Label(keys, text=text).pack(side='left', padx=(0, 3))
            variable = tk.StringVar(value=value)
            self.key_vars.append(variable)
            ttk.Entry(keys, textvariable=variable, width=4).pack(side='left', padx=(0, 7))
        self.calibration_var = tk.StringVar(value='已保存校准' if self.settings.calibration else '尚未校准')
        ttk.Button(sidebar, text='原地转向校准', command=self.calibrate).pack(fill='x')
        ttk.Label(sidebar, textvariable=self.calibration_var, style='Hint.TLabel', wraplength=275).pack(anchor='w', pady=4)
        ttk.Separator(action_panel).pack(fill='x', pady=(0, 8))
        self.start_button = ttk.Button(action_panel, text='开始 · 从实时位置重新规划', style='Accent.TButton', command=self.start)
        self.start_button.pack(fill='x', pady=(0, 6))
        controls = ttk.Frame(action_panel)
        controls.pack(fill='x')
        ttk.Button(controls, text='暂停 / 继续 F8', command=lambda: self.send('toggle')).pack(side='left')
        ttk.Button(controls, text='停止 F12', command=lambda: self.send('stop')).pack(side='right')
        self.status_var = tk.StringVar(value='离线预览不连接设备' if self.demo else '等待游戏窗口与有效色块')
        ttk.Label(action_panel, textvariable=self.status_var, wraplength=275).pack(anchor='w', pady=(8, 0))
        def bind_sidebar_scroll(widget):
            widget.bind('<MouseWheel>', lambda e: sidebar_canvas.yview_scroll(-1 if e.delta > 0 else 1, 'units'))
            for child in widget.winfo_children():
                bind_sidebar_scroll(child)
        bind_sidebar_scroll(sidebar)
        bottom = ttk.Frame(self.root, padding=(18, 0, 18, 12))
        bottom.pack(fill='x')
        self.messages = tk.Text(bottom, height=4, relief='flat', bg='#e3eaee', fg='#314d5b',
                                font=('Microsoft YaHei UI', 9), state='disabled', wrap='word')
        self.messages.pack(fill='x')
        self.map.candidates = self.candidates_var.get()
        self.refresh_windows()

    def log(self, text):
        self.messages.configure(state='normal')
        self.messages.insert('end', text + '\n')
        if int(self.messages.index('end-1c').split('.')[0]) > 250:
            self.messages.delete('1.0', '50.0')
        self.messages.see('end')
        self.messages.configure(state='disabled')

    def refresh_windows(self):
        try:
            self.window_combo['values'] = [title for _, title in list_game_windows()]
        except (OSError, RuntimeError) as exc:
            self.events.put({'kind': 'message', 'text': f'窗口列表读取失败：{exc}'})

    def hover(self, point):
        self.hover_var.set(point_text(point) + '   · 单击 B / Shift+单击 A / 拖动 / 滚轮缩放')

    def pick(self, point, start=False):
        if self.state in ACTIVE_STATES:
            self.log('请先暂停当前任务，再修改起终点。')
            return
        self.cancel_preview()
        name = 'A' if start else 'B'
        self.height_vars[name].set('自动（单层地面）')
        self.height_boxes[name]['values'] = ['自动（单层地面）']
        self.height_values[name] = {}
        if start:
            self.preview_start = point
            self.map.start = point
            self.start_var.set('A  ' + point_text(point))
        else:
            self.map.goal = point
            self.goal_var.set('B  ' + point_text(point))
        self.map.set_route(None)
        self.route_var.set('目的地已更新，请预览；执行时仍使用实时起点')
        generation = self.preview_generation
        def inspect_height():
            try:
                from .navmesh import load_pack
                pack = load_pack(self.bundle.asset_root)
                if pack:
                    heights = pack.heights(point)
                    self.events.put({'kind': 'heights', 'name': name, 'heights': heights,
                                     'point': point, 'generation': generation})
            except Exception as exc:
                self.events.put({'kind': 'height_error', 'text': str(exc), 'generation': generation})
        self.executor.submit(inspect_height)

    def select_height(self, name):
        if self.state in ACTIVE_STATES:
            self.log('请先暂停当前任务，再修改地面高度。')
            return
        point = self.preview_start if name == 'A' else self.map.goal
        if point is None:
            return
        z = self.height_values[name].get(self.height_vars[name].get())
        point = Point(point.c, point.x, point.y, z)
        if name == 'A':
            self.preview_start = self.map.start = point
            self.start_var.set('A  ' + point_text(point))
        else:
            self.map.goal = point
            self.goal_var.set('B  ' + point_text(point))
        self.cancel_preview()
        self.map.set_route(None)
        self.route_var.set('地面高度已改变，请重新预览')

    def clear_start(self):
        if self.state in ACTIVE_STATES:
            self.log('请先暂停当前任务，再修改起点。')
            return
        self.preview_start = None
        self.map.start = None
        self.start_var.set('A  使用实时位置')
        self.height_vars['A'].set('自动（单层地面）')
        self.cancel_preview()
        self.map.set_route(None)

    def change_policy(self):
        if self.runtime and self.state in ACTIVE_STATES:
            self.runtime.command('pause')
        self.map.candidates = self.candidates_var.get()
        self.cancel_preview()
        self.map.set_route(None)
        self.map.schedule_redraw()
        self.route_var.set('道路策略已改变，请重新预览')

    def cancel_preview(self):
        self.preview_generation += 1
        self.preview_cancel.set()
        if self.preview_future:
            self.preview_future.cancel()

    def preview(self):
        if self.state in ACTIVE_STATES:
            self.log('请先暂停当前任务，再预览其他路线。')
            return
        if not self.map.goal:
            self.log('请先在地图点击终点 B。')
            return
        start = self.preview_start or (self.latest.point if self.latest and self.latest.flags & Flags.VALID else None)
        if not start:
            self.log('尚无实时位置，请 Shift+单击地图设预览起点 A。')
            return
        self.cancel_preview()
        generation = self.preview_generation
        self.preview_cancel = threading.Event()
        cancel = self.preview_cancel
        goal, allow = self.map.goal, self.candidates_var.get()
        self.route_var.set('正在规划，可重新选点取消…')
        def work():
            try:
                route = Planner(self.bundle).plan(start, goal, allow_candidates=allow, cancel=cancel)
                self.events.put({'kind': 'preview', 'route': route, 'generation': generation})
            except Exception as exc:
                self.events.put({'kind': 'preview_error', 'text': str(exc), 'generation': generation})
        self.preview_future = self.executor.submit(work)

    def _apply_settings(self):
        values = [v.get().strip().upper() for v in self.key_vars]
        changed_binding = values != [self.settings.forward_key, self.settings.left_key, self.settings.right_key] or \
            self.window_var.get().strip() != self.settings.window_title
        settings = Settings(host=self.host_var.get().strip(), port=int(self.port_var.get()),
                            window_title=self.window_var.get().strip(), forward_key=values[0],
                            left_key=values[1], right_key=values[2], allow_candidates=self.candidates_var.get(),
                            calibration=None if changed_binding else self.settings.calibration)
        settings.validate()
        if settings != self.settings:
            settings.save()
            self.settings = settings
            self.device_diagnostic_var.set('尚未读取固件版本与能力')
            if self.runtime:
                self.runtime.command('configure', settings=settings)
        return settings

    def send(self, command):
        if self.runtime:
            self.runtime.command(command)
        else:
            self.log('当前为离线模式，不发送设备指令。')

    def diagnose(self):
        try:
            self._apply_settings()
            self.send('diagnose')
        except (ValueError, OSError) as exc:
            self.log(str(exc))

    def calibrate(self):
        try:
            self._apply_settings()
            if self.runtime:
                self.state = 'calibration_armed'
            self.send('calibrate')
        except (ValueError, OSError) as exc:
            self.log(str(exc))

    def start(self):
        if not self.map.goal:
            self.log('请先选择终点 B。')
            return
        try:
            self._apply_settings()
            if self.runtime:
                self.cancel_preview()
                self.state = 'armed'
                self.runtime.command('arm', goal=self.map.goal, allow_candidates=self.candidates_var.get())
            else:
                self.log('离线模式不能开始实际行走。请用正常启动方式打开客户端。')
        except (ValueError, OSError) as exc:
            self.log(str(exc))

    def _show_route(self, route):
        self.map.set_route(route)
        estimated = sum(s.distance for s in route.segments if s.reliability in ('candidate', 'terrain'))
        precise = sum(s.distance for s in route.segments if s.reliability == 'navmesh')
        self.route_var.set(f'总长 {route.distance:,.0f} 码\n静态碰撞导航 {precise:,.0f} 码\n候选 / 地形估算 {estimated:,.0f} 码\n搜索展开 {route.expanded:,} 次')

    def poll(self):
        if self.closed:
            return
        pose_changed = False
        try:
            # Bound each UI batch. Keep every trail sample, but render once;
            # a backlog must not multiply expensive track redraws or monopolize
            # the Tk event loop while the game and control worker are active.
            for _ in range(200):
                event = self.events.get_nowait()
                kind = event['kind']
                if kind == 'heights':
                    name = event['name']
                    point = self.preview_start if name == 'A' else self.map.goal
                    if point == event['point']:
                        values = {f'高度 {h:.2f} 码': h for h in event['heights']}
                        self.height_values[name] = values
                        self.height_boxes[name]['values'] = ['自动（单层地面）', *values]
                        if len(values)>1:
                            self.log(f'{name} 有 {len(values)} 层可步行地面，请在高度列表选择。')
                        elif not values:
                            self.log(f'{name} 不在已知可步行表面内，请调整选点。')
                    continue
                if kind == 'height_error':
                    if event['generation'] == self.preview_generation:
                        self.log(event['text'])
                    continue
                if kind == 'telemetry':
                    self.latest = event['telemetry']
                    if self.latest.flags & Flags.VALID:
                        self.map.update_pose(self.latest, draw=False)
                        pose_changed = True
                elif kind in ('preview', 'preview_error'):
                    if event['generation'] == self.preview_generation:
                        if kind == 'preview':
                            self._show_route(event['route'])
                        else:
                            self.route_var.set(event['text'])
                            self.log('路线未生成：' + event['text'])
                elif kind == 'route':
                    self.map.start = event['route'].start
                    self.start_var.set('A  执行起点 ' + point_text(self.map.start))
                    self._show_route(event['route'])
                elif kind == 'message':
                    self.log(event['text'])
                elif kind == 'device_diagnostic':
                    self.device_diagnostic_var.set(event['summary'])
                    self.log(event['summary'].replace('\n', '；') + '\n' + event['details'])
                elif kind == 'device_diagnostic_reset':
                    self.device_diagnostic_var.set('尚未读取固件版本与能力')
                elif kind == 'status':
                    self.state = event['state']
                    self.state_var.set(STATE_NAMES.get(self.state, self.state))
                    self.status_var.set(event['reason'] + '\n' + event['capture'])
                elif kind == 'calibrated':
                    self.settings.calibration = event['calibration']
                    self.calibration_var.set('校准已保存；修改转向速度或按键后需重做')
        except queue.Empty:
            pass
        if pose_changed:
            self.map.draw_pose()
        self.root.after(40, self.poll)

    def choose_replay(self):
        path = filedialog.askopenfilename(title='选择 JSONL 运行记录', initialdir=str(ROOT / 'runtime'),
                                          filetypes=[('运行记录', '*.jsonl'), ('所有文件', '*')])
        if path:
            if self.runtime:
                # Stop live input before displaying a separate immutable replay.
                self.runtime.command('stop')
                child = tk.Toplevel(self.root)
                Application(child, self.bundle, demo=True, replay_path=path)
            else:
                self.load_replay(path)

    def load_replay(self, path):
        try:
            replay = Replay.load(path)
            self.log('回放摘要：' + json.dumps(replay.summary(), ensure_ascii=False))
            if self.replay_job:
                self.root.after_cancel(self.replay_job)
            self.replay_rows = replay.rows
            self.replay_index = 0
            self.map.track.clear()
            # Let initial layout establish actual canvas dimensions before fit.
            self.replay_job = self.root.after(100, self._replay_next)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.log(f'回放读取失败：{exc}')

    def _replay_next(self):
        if self.closed or self.replay_index >= len(self.replay_rows):
            return
        row = self.replay_rows[self.replay_index]
        self.replay_index += 1
        if row['kind'] == 'telemetry':
            t = telemetry_from_dict(row['telemetry'])
            self.latest = t
            if t.flags & Flags.VALID:
                self.map.update_pose(t)
        elif row['kind'] == 'route':
            route = route_from_dict(row['route'])
            self.map.start, self.map.goal = route.start, route.goal
            self.start_var.set('A  回放起点 ' + point_text(route.start))
            self.goal_var.set('B  回放终点 ' + point_text(route.goal))
            self.continent.set('东部王国' if route.start.c == 0 else '卡利姆多')
            self.map.view.c, self.map.view.north, self.map.view.west = route.start.c, route.start.x, route.start.y
            self.map.view.scale = .7
            self._show_route(route)
            self.map.fit_route()
            self.map.schedule_redraw()
        elif row['kind'] == 'state':
            self.state_var.set('回放 · ' + STATE_NAMES.get(row.get('state'), row.get('state', '')))
        next_time = self.replay_rows[self.replay_index].get('t', 0) if self.replay_index < len(self.replay_rows) else row.get('t', 0)
        delay = min(1000, max(1, int((next_time - row.get('t', 0)) * 1000)))
        self.replay_job = self.root.after(delay, self._replay_next)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.cancel_preview()
        if self.replay_job:
            self.root.after_cancel(self.replay_job)
        if self.hotkeys:
            self.hotkeys.close()
        if self.runtime:
            self.runtime.close()
        self.executor.shutdown(wait=False, cancel_futures=True)
        self.root.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description='Forever 地面导航客户端')
    parser.add_argument('--demo', action='store_true', help='仅离线地图预览，不捕获游戏，不连接 Pico')
    parser.add_argument('--replay', type=Path, help='离线回放 JSONL 日志')
    parser.add_argument('--data', type=Path, default=ROOT / 'data')
    args = parser.parse_args(argv)
    bundle = DataBundle.load(args.data)
    enable_dpi_awareness()
    root = tk.Tk()
    Application(root, bundle, demo=args.demo, replay_path=args.replay)
    root.mainloop()


if __name__ == '__main__':
    main()
