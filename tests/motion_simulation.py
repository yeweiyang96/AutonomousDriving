"""Independent offline game plant for recorded production motion regressions.

Only the controller uses its own prediction model. Actual movement uses midpoint
integration, measured turn/strafe parameters and native ground surface heights.
Firmware window replacement and delayed game input have separate timelines.
No capture, network or Pico objects are created; no runtime fixtures are read.
"""
from collections import deque
from dataclasses import asdict, replace
import heapq
import hashlib
import math
from pathlib import Path
import sys
import time

from adrive.controller import Calibration
from adrive.models import Point
from adrive.motion import MotionController, MotionProfile, MotionTrajectory
from adrive.motion_worker import SurfaceValidator
from adrive.replay import route_from_dict, telemetry_from_dict

ROOT = Path(__file__).resolve().parents[1]
BINDING_NAMES = ('forward', 'turn_left', 'turn_right', 'strafe_left', 'strafe_right', 'walk_toggle')
SOURCE_FILES = ('adrive/motion.py', 'adrive/motion_worker.py', 'adrive/navmesh.py',
                'adrive/motion_protocol.py', 'tests/motion_simulation.py')


def source_hashes():
    return {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in SOURCE_FILES}


def brief(p, facing, speed):
    return dict(c=p.c, x=p.x, y=p.y, z=p.z, facing=facing,
                facing_degrees=math.degrees(facing), speed=speed)


def run_recorded_case(fixture, walk_speed=5., max_step=.005, target_distance=40., *, debug=False, initial_walking=False):
    source_sha256 = source_hashes()
    bindings = tuple(fixture['bindings'][name] for name in BINDING_NAMES)
    forward, turn_left, turn_right, strafe_left, strafe_right, walk_toggle = bindings
    moving = {forward, strafe_left, strafe_right}
    route = route_from_dict(fixture['route'])
    recorded = telemetry_from_dict(fixture['telemetry'])
    calibration = fixture['calibration']
    profile = MotionProfile.from_calibration(Calibration(**calibration['calibration']),
                                             calibration['movement_calibration'])
    actual_yaw = (profile.left_rate, profile.right_rate)
    actual_offsets = (profile.strafe_left_angle, profile.strafe_right_angle,
                      profile.diagonal_left_angle, profile.diagonal_right_angle)
    latency = profile.latency
    validator = SurfaceValidator(ROOT/'data/navigation/manifest.json')
    controller = MotionController(profile, bindings, validator, route, 1)
    original = route.points
    wall_start = time.monotonic()
    now = 0.; next_observation = 0.; sequence = recorded.seq
    p = recorded.point; facing = recorded.facing
    fw_keys = (); game_keys = (); fw_schedule = []; game_schedule = []; schedule_id = 0
    applied_event = 0; walking = initial_walking; game_toggles = 0
    distance = 0.; observed_count = 0; moving_started = False
    all_points = [p]; corridor_refs = set(); local_points = [p]
    failures = []; windows = []; responses = []; actual_changes = []
    window_durations = []; controller_ms = []; expired_windows = 0
    certified_steps = 0; source_speed = recorded.ground_speed
    max_lateral = 0.; max_vertical = 0.; measured_lateral_points = 0
    max_lateral_at = None; max_lateral_after_join = 0.
    pivot_windows = 0; release_windows = 0; distinct_events = set()
    project_calls = deque(maxlen=20); failure_locals = {}; last_project_calls = []
    last_reply = None
    original_project = MotionTrajectory.project

    def traced_project(trajectory, point, progress, span):
        result = original_project(trajectory, point, progress, span)
        project_calls.append(dict(point=asdict(point), progress=progress, span=span,
                                  result=None if result is None else dict(error=result[0], progress=result[1], point=asdict(result[2]))))
        return result

    def capture_failure(frame, event, arg):
        if event == 'return' and frame.f_code is MotionController.update.__code__ and getattr(arg, 'kind', 'window') != 'window':
            values = frame.f_locals
            for name in ('progress', 's', 'score', 'cross', 'heading', 'next_heading', 'facing', 'error', 'keys', 'side', 'desired', 'delay', 'dt', 'measured', 'expected'):
                if name in values: failure_locals[name] = values[name]
            for name in ('p', 'predicted', 'q', 'reference', 'ground', 'target', 'old'):
                if name in values: failure_locals[name] = asdict(values[name])
            if values.get('scored'):
                failure_locals['best_candidates'] = [dict(score=v[0], keys=v[1], point=asdict(v[2]), facing=v[3], progress=v[4])
                                                     for v in sorted(values['scored'], key=lambda v:v[0])[:5]]
            if 'remaining' in values:
                failure_locals['remaining'] = [asdict(point) for point in values['remaining']]
            if 'trajectory' in values:
                failure_locals['candidate_trajectory'] = asdict(values['trajectory'])

    if debug:
        MotionTrajectory.project = traced_project

    def ground(x, y, z):
        heights = validator.mesh.heights(Point(p.c, x, y, z))
        candidates = [h for h in heights if abs(h-z) <= .5]
        if not candidates:
            raise RuntimeError(f'actual ground unavailable: {x}, {y}, {z}; heights={heights}')
        return min(candidates, key=lambda h: abs(h-z))

    def firmware_change(keys, at):
        nonlocal fw_keys, schedule_id
        keys = tuple(sorted(keys))
        if keys == fw_keys:
            return
        fw_keys = keys
        schedule_id += 1
        heapq.heappush(game_schedule, (at+latency, schedule_id, keys))

    def snapshot(reason, kind):
        projected = controller.trajectory.project(p, controller.progress, 12.) if controller.trajectory else None
        return dict(kind=kind, reason=reason, at=now, travelled=distance,
                    pose=brief(p, facing, ((walk_speed if walking else source_speed) if moving.intersection(game_keys) else 0.)),
                    firmware_keys=fw_keys, game_keys=game_keys, applied_event=applied_event,
                    walking=walking, reference_index=controller.reference_index,
                    progress=controller.progress, joining_until=controller.joining_until,
                    trajectory_length=controller.trajectory.length if controller.trajectory else None,
                    projection=None if projected is None else dict(error=projected[0], distance=projected[1], point=asdict(projected[2])),
                    slow_requested=controller.slow_requested, slow_applied=controller.slow_applied,
                    slow_deadline=controller.slow_deadline, speed_mismatches=controller.speed_mismatches)

    try:
        while now <= 30. and distance < target_distance:
            # Expired/replaced firmware segments never get replayed. Game input
            # reports already submitted through USB retain their input delay.
            while fw_schedule and fw_schedule[0][0] <= now+1e-10:
                _, keys, expired = fw_schedule.pop(0)
                expired_windows += int(expired)
                firmware_change(keys, now)
            while game_schedule and game_schedule[0][0] <= now+1e-10:
                _, _, keys = heapq.heappop(game_schedule)
                if walk_toggle in keys and walk_toggle not in game_keys:
                    walking = not walking; game_toggles += 1
                game_keys = keys
                actual_changes.append(dict(at=now, keys=keys, walking=walking))
                if moving.intersection(keys):
                    moving_started = True
                elif moving_started:
                    failures.append(snapshot('all-release or pivot reached the game before a requested stop', 'input_continuity'))
                    break
            if failures: break

            if now >= next_observation-1e-10:
                i = controller.reference_index
                end = min(i+1, len(original)-1)
                local_distance = p.distance(original[end])
                live_speed = (walk_speed if walking else source_speed) if moving.intersection(game_keys) else 0.
                prediction_speed = live_speed or source_speed
                while end < len(original)-1 and local_distance < max(24., prediction_speed*4):
                    local_distance += original[end].distance(original[end+1]); end += 1
                validator.prepare(tuple(original[max(0, i-1):end+1]))
                corridor_refs.update(validator.corridor)
                if len(local_points)>1:
                    # Every integration chord is swept; no sparse endpoint-only
                    # or controller-predicted trace can certify the true path.
                    if not validator.mesh.motion_validate(tuple(local_points), .1001, tuple(corridor_refs)):
                        failures.append(snapshot('independent physical trace failed swept ground/corridor certification', 'actual_collision'))
                        break
                    certified_steps += len(local_points)-1
                    local_points = [p]
                x = math.floor(p.x*16+.5)/16
                y = math.floor(p.y*16+.5)/16
                z = ground(x, y, p.z)
                heading = (math.floor((facing%math.tau)/math.tau*65536+.5)%65536)/65536*math.tau
                sequence += 1
                t = replace(recorded, seq=sequence, captured_at=100.+now, x=x, y=y, z=z,
                            facing=heading, speed=math.floor(live_speed*256+.5)/256)
                began = time.monotonic()
                project_calls.clear()
                if debug:
                    sys.setprofile(capture_failure)
                try:
                    w = controller.update(t, 100.+now, fw_keys, applied_event)
                finally:
                    if debug:
                        sys.setprofile(None)
                last_project_calls = list(project_calls)
                last_reply = w
                controller_ms.append((time.monotonic()-began)*1000)
                observed_count += 1
                responses.append(dict(at=now, kind=w.kind, reason=w.reason, event=w.event_id,
                                      pose=brief(t.point, t.facing, t.speed),
                                      actual_keys=fw_keys, reference_index=controller.reference_index,
                                      progress=controller.progress, joining_until=controller.joining_until))
                if w.kind == 'wait':
                    # Keep already accepted firmware segments and game reports.
                    # In particular, do not extend the old window's deadline or
                    # replay its one-shot while waiting for a measured speed.
                    next_observation += .033
                    continue
                if w.kind != 'window':
                    failures.append(snapshot(w.reason, w.kind)); break
                duration = sum(f.duration_ms for f in w.frames)
                window_durations.append(duration)
                if not 0 < duration <= 200 or len(w.frames)>16 or any(f.duration_ms<20 for f in w.frames):
                    failures.append(snapshot('invalid firmware window duration/capacity', 'window_contract')); break
                if any(not moving.intersection(f.keys) for f in w.frames):
                    pivot_windows += sum(bool(set(f.keys).intersection((turn_left, turn_right))) and not moving.intersection(f.keys) for f in w.frames)
                    release_windows += sum(not f.keys for f in w.frames)
                    failures.append(snapshot('window contains all-release or stationary turn', 'window_contract')); break
                if w.event_id:
                    if applied_event not in (0, w.event_id):
                        failures.append(snapshot('walk action id changed unexpectedly', 'one_shot')); break
                    applied_event = w.event_id
                    distinct_events.add(w.event_id)
                windows.append(dict(at=now, event=w.event_id, frames=[asdict(f) for f in w.frames],
                                    reason=w.reason, margin=w.margin))
                fw_schedule = []
                firmware_change(w.frames[0].keys, now)
                cursor = now
                for index, frame in enumerate(w.frames):
                    cursor += frame.duration_ms/1000
                    if index+1 < len(w.frames):
                        fw_schedule.append((cursor, w.frames[index+1].keys, False))
                    else:
                        fw_schedule.append((cursor, (), True))
                next_observation += .033

            boundary = min(next_observation,
                           fw_schedule[0][0] if fw_schedule else math.inf,
                           game_schedule[0][0] if game_schedule else math.inf)
            dt = min(max_step, boundary-now)
            if dt <= 1e-10:
                now = boundary
                continue
            # Independent midpoint integration: no propagate(), trajectory
            # samples, predicted poses, or controller local heights are used.
            turn_rate = (actual_yaw[0] if turn_left in game_keys else 0.)-(actual_yaw[1] if turn_right in game_keys else 0.)
            travel_offset = 0.
            if strafe_left in game_keys: travel_offset = actual_offsets[2] if forward in game_keys else actual_offsets[0]
            if strafe_right in game_keys: travel_offset = actual_offsets[3] if forward in game_keys else actual_offsets[1]
            if moving.intersection(game_keys):
                speed = walk_speed if walking else source_speed
                direction = facing+turn_rate*dt/2+travel_offset
                ux, uy = math.cos(direction), math.sin(direction)
                ahead_z = ground(p.x+.1*ux, p.y+.1*uy, p.z)
                slope = (ahead_z-p.z)/.1
                horizontal_speed = speed/math.sqrt(1+slope*slope)
                nx, ny = p.x+ux*horizontal_speed*dt, p.y+uy*horizontal_speed*dt
                nz = ground(nx, ny, p.z)
                nxt = Point(p.c, nx, ny, nz)
                distance += p.distance(nxt)
                p = nxt; all_points.append(p); local_points.append(p)
                if controller.trajectory:
                    projection = original_project(controller.trajectory, p, controller.progress, max(12., source_speed*.4))
                    if projection is not None:
                        in_join = projection[1] < controller.joining_until
                        if projection[0] > max_lateral:
                            max_lateral_at = dict(at=now+dt, travelled=distance, point=asdict(p),
                                                 facing=(facing+turn_rate*dt)%math.tau,
                                                 progress=projection[1], joining_until=controller.joining_until,
                                                 in_join=in_join, reference_index=controller.reference_index)
                        max_lateral = max(max_lateral, projection[0])
                        if not in_join:
                            max_lateral_after_join = max(max_lateral_after_join, projection[0])
                        max_vertical = max(max_vertical, abs(p.z-projection[2].z))
                        measured_lateral_points += 1
            facing = (facing+turn_rate*dt)%math.tau
            now += dt
        if len(local_points)>1:
            if not validator.mesh.motion_validate(tuple(local_points), .1001, tuple(corridor_refs)):
                failures.append(snapshot('last physical trace failed swept certification', 'actual_collision'))
            else: certified_steps += len(local_points)-1
        if not failures and distance<target_distance:
            failures.append(snapshot('30-second simulation budget reached', 'simulation_timeout'))
        final = snapshot('target distance reached' if not failures else failures[0]['reason'], 'passed' if not failures else 'failed')
        result = dict(walk_speed=walk_speed, initial_walking=initial_walking, bindings=bindings, integration_step_ms=max_step*1000,
                      simulated_seconds=now, travelled_yards=distance, target_yards=target_distance,
                      outcome=final['kind'], first_response=responses[0] if responses else None,
                      first_failure=failures[0] if failures else None, final=final,
                      applied_event=applied_event, game_walk_toggles=game_toggles,
                      distinct_one_shot_events=len(distinct_events),
                      pivot_windows=pivot_windows, release_windows=release_windows,
                      max_lateral_error=max_lateral, max_vertical_route_error=max_vertical,
                      max_lateral_at=max_lateral_at, max_lateral_after_join=max_lateral_after_join,
                      lateral_samples=measured_lateral_points,
                      windows=len(windows), max_window_ms=max(window_durations, default=0),
                      wait_responses=sum(r['kind']=='wait' for r in responses),
                      expired_windows=expired_windows, physical_points=len(all_points),
                      certified_chords=certified_steps, full_actual_trace_certified=certified_steps==len(all_points)-1,
                      controller_wall_ms_total=sum(controller_ms), controller_wall_ms_max=max(controller_ms, default=0),
                      wall_seconds=time.monotonic()-wall_start,
                      controller_profile=asdict(controller.profile),
                      controller_trajectory=None if controller.trajectory is None else asdict(controller.trajectory),
                      reply_predicted_start=asdict(last_reply.predicted_start) if last_reply else None,
                      failure_update_locals=failure_locals, failure_project_calls=last_project_calls,
                      source_sha256=source_sha256, source_sha256_after=source_hashes(),
                      responses=responses, window_details=windows, game_input_changes=actual_changes,
                      physical_trace=[asdict(q) for q in all_points])
        return result
    finally:
        if debug:
            MotionTrajectory.project = original_project
        validator.close()

