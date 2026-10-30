from dataclasses import replace
import math
import unittest

from adrive.models import Point,Route,Segment,Telemetry,Flags,PlanningError,angle_error
from adrive.motion import MotionController,MotionProfile,smooth_route,propagate,join_trajectory

PROFILE=MotionProfile(math.pi,math.pi,strafe_verified=True)
BINDINGS=(26,4,7,20,8,84)


def route(*xy):
    pts=tuple(Point(0,x,y,0) for x,y in xy)
    return Route(pts[0],pts[-1],tuple(Segment((a,b),'fixture','navmesh',str(i)) for i,(a,b) in enumerate(zip(pts,pts[1:]))))


def tight_route():
    # A sharp short bend needs a smaller radius even on otherwise open ground.
    # An unobstructed short right angle can instead be safely shortened.
    angle=math.radians(100)
    return route((0,0),(2,0),(2+2*math.cos(angle),2*math.sin(angle)))


def simulate(r,speed=lambda i:7.,limit=6000,initial_heading=None):
    controller=MotionController(PROFILE,BINDINGS,lambda points,margin:True,r)
    p=r.start;heading=math.atan2(r.points[1].y-p.y,r.points[1].x-p.x)
    if initial_heading is not None:heading=initial_heading
    keys=();frames=[];due=0;trace=[];windows=[]
    for i in range(limit):
        v=speed(i);now=10+i*.02
        t=Telemetry(1,i+1,p.c,1,p.x,p.y,heading,v,Flags.VALID,now,z=0,ground_speed=v)
        if i%2==0:
            result=controller.update(t,now,keys)
            if result.kind in ('arrived','settle'):return trace,windows,controller
            if result.kind!='window':raise AssertionError((i,result.kind,result.reason,p,controller.progress))
            windows.append(result);frames=list(result.frames);due=now
        if now>=due-1e-8 and frames:
            frame=frames.pop(0);keys=frame.keys;due=now+frame.duration_ms/1000
        if not keys or not any(k in keys for k in (26,20,8)):raise AssertionError('involuntary stop/pivot')
        # Independent 1 ms integration, rather than calling the planner's
        # analytic predictor to manufacture the observations it expects.
        offset=(math.pi/4 if 26 in keys else math.pi/2) if 20 in keys else (-math.pi/4 if 26 in keys else -math.pi/2) if 8 in keys else 0
        omega=(math.pi if 4 in keys else 0)-(math.pi if 7 in keys else 0)
        for _ in range(20):
            h=heading+omega*.0005
            p=replace(p,x=p.x+v*.001*math.cos(h+offset),y=p.y+v*.001*math.sin(h+offset))
            heading=(heading+omega*.001)%math.tau
        trace.append(p)
    raise AssertionError(('did not arrive',p,controller.progress))


class GeometryTests(unittest.TestCase):
    def test_fillet_tangent_entry_exit_and_radius(self):
        t=smooth_route(route((0,0),(20,0),(20,20)).points,7,PROFILE,lambda p,m:True)
        a,arc,b=t.primitives
        self.assertAlmostEqual(arc.radius,7/math.pi*1.15)
        self.assertEqual(a.end,arc.start);self.assertEqual(arc.end,b.start)
        self.assertLess(abs(angle_error(a.heading(1),arc.heading(0))),1e-9)
        self.assertLess(abs(angle_error(arc.heading(1),b.heading(0))),1e-9)
        self.assertAlmostEqual(arc.start.y,0);self.assertAlmostEqual(arc.end.x,20)

    def test_short_jitter_merges_only_when_entire_line_is_clear(self):
        r=route((0,0),(1,.01),(2,-.01),(4,0))
        t=smooth_route(r.points,7,PROFILE,lambda p,m:True)
        self.assertEqual(len(t.primitives),1)
        with self.assertRaises(PlanningError):smooth_route(r.points,7,PROFILE,lambda p,m:False)

    def test_lateral_motion_is_normalized(self):
        p=Point(0,0,0,0)
        for keys in ((26,20),(26,8),(20,),(8,)):
            q,h=propagate(p,0,7,keys,1,PROFILE,BINDINGS)
            self.assertAlmostEqual(p.distance(q),7)
            self.assertEqual(h,0)

    def test_continuous_straight_left_right_and_s_turns(self):
        for xy in (((0,0),(100,0)),((0,0),(30,0),(30,30)),((0,0),(30,0),(30,-30)),
                   ((0,0),(30,0),(30,30),(60,30),(60,60))):
            with self.subTest(xy=xy):
                r=route(*xy);trace,windows,c=simulate(r)
                self.assertLessEqual(trace[-1].distance(r.goal),.6)
                self.assertTrue(all(sum(f.duration_ms for f in w.frames)<=200 for w in windows))

    def test_runtime_speed_increases_and_decreases(self):
        r=route((0,0),(40,0),(40,40),(90,40))
        trace,windows,c=simulate(r,lambda i:7 if i<160 else 14 if i<300 else 3.5)
        self.assertLessEqual(trace[-1].distance(r.goal),.6)
        self.assertEqual(c.last_speed,3.5)

    def test_standing_start_requires_live_ground_speed(self):
        c=MotionController(PROFILE,BINDINGS,lambda p,m:True,route((0,0),(100,0)))
        t=Telemetry(1,1,0,1,0,0,0,0,Flags.VALID,10,z=0)
        self.assertEqual(c.update(t,10).kind,'stop')
        c=MotionController(PROFILE,BINDINGS,lambda p,m:True,route((0,0),(100,0)))
        self.assertEqual(c.update(replace(t,ground_speed=11.2),10).kind,'window')

    def test_legacy_airborne_and_old_frames_cannot_move(self):
        for values in ({'protocol_version':3},{'environment':12},{'captured_at':9}):
            c=MotionController(PROFILE,BINDINGS,lambda p,m:True,route((0,0),(100,0)))
            t=Telemetry(1,1,0,1,0,0,0,7,Flags.VALID,10,z=0)
            self.assertEqual(c.update(replace(t,**values),10).kind,'stop')

    def test_loop_preserves_order_and_corner_tracking_stays_within_point_three(self):
        r=route((0,0),(30,0),(30,30),(0,30),(0,5))
        trace,windows,c=simulate(r)
        self.assertGreater(len(trace)*.02,r.distance/7*.85)
        reference=smooth_route(r.points,7,PROFILE,lambda p,m:True)
        progress=0
        for p in trace:
            error,along,_=reference.project(p,progress,3)
            self.assertLessEqual(error,.3)
            progress=max(progress,along)

    def test_strafe_joins_a_route_without_pivot_or_stopping(self):
        c=MotionController(PROFILE,BINDINGS,lambda p,m:True,route((0,0),(30,0)))
        t=Telemetry(1,1,0,1,0,0,math.pi/2,7,Flags.VALID,10,z=0,ground_speed=7)
        w=c.update(t,10)
        self.assertEqual(w.kind,'window')
        self.assertIn(8,w.frames[0].keys)

    def test_narrow_corner_requests_walk_once_and_awaits_measurement(self):
        c=MotionController(PROFILE,BINDINGS,lambda p,m:True,tight_route())
        t=Telemetry(1,1,0,1,0,0,0,7,Flags.VALID,10,z=0,ground_speed=7)
        first=c.update(t,10);self.assertGreater(first.event_id,0)
        self.assertIn(84,first.frames[0].keys)
        waiting=c.update(replace(t,seq=2,captured_at=10.04,x=.28),10.04,(26,),first.event_id)
        self.assertEqual(waiting.event_id,0)
        slow=c.update(replace(t,seq=3,captured_at=10.08,x=.38,speed=2.5),10.08,(26,),first.event_id)
        self.assertEqual(slow.kind,'window',slow.reason)
        self.assertTrue(all(84 not in f.keys for f in slow.frames))

    def test_uncertified_corner_stops_without_inventing_a_blocked_road(self):
        c=MotionController(PROFILE,BINDINGS,lambda p,m:False,route((0,0),(3,0),(3,3)))
        t=Telemetry(1,1,0,1,0,0,0,7,Flags.VALID,10,z=0,ground_speed=7)
        w=c.update(t,10)
        self.assertEqual(w.kind,'stop');self.assertFalse(w.frames);self.assertIsNone(w.blocked_edge)

    def test_backwards_start_joins_with_continuous_motion_and_tangency(self):
        r=route((0,0),(60,0))
        t=Telemetry(1,1,0,1,0,0,math.pi,14,Flags.VALID,10,z=0,ground_speed=14)
        base=smooth_route(r.points,14,PROFILE,lambda p,m:True)
        joined,length=join_trajectory(base,t,14,PROFILE,lambda p,m:True)
        self.assertGreater(length,0)
        for a,b in zip(joined.primitives,joined.primitives[1:]):
            self.assertLess(a.end.distance(b.start),1e-7)
            self.assertLess(abs(angle_error(a.heading(1),b.heading(0))),1e-7)
        trace,windows,c=simulate(r,lambda i:14.,initial_heading=math.pi)
        self.assertLessEqual(trace[-1].distance(r.goal),.6)
        self.assertTrue(all(any(k in f.keys for k in (26,20,8)) for w in windows for f in w.frames))

    def test_collinear_portal_splits_do_not_shrink_a_real_corner(self):
        r=route((0,0),(20,0),(20,1),(20,2),(20,20))
        trajectory=smooth_route(r.points,14,PROFILE,lambda p,m:True)
        self.assertEqual(len(trajectory.primitives),3)

    def test_unsubmitted_walk_event_is_reoffered_with_same_id(self):
        c=MotionController(PROFILE,BINDINGS,lambda p,m:True,tight_route())
        t=Telemetry(1,1,0,1,0,0,0,7,Flags.VALID,10,z=0,ground_speed=7)
        first=c.update(t,10)
        second=c.update(replace(t,seq=2,captured_at=10.2),10.2)
        self.assertGreater(first.event_id,0);self.assertEqual(second.event_id,first.event_id)
        waiting=c.update(replace(t,seq=3,captured_at=10.24),10.24,first.frames[-1].keys,first.event_id)
        self.assertEqual(waiting.event_id,0)
        self.assertTrue(all(84 not in f.keys for f in waiting.frames))

    def test_stationary_unknown_walk_mode_probes_before_any_toggle(self):
        for toggle_allowed in (False,True):
            with self.subTest(toggle_allowed=toggle_allowed):
                profile=replace(PROFILE,walk_toggle_allowed=toggle_allowed)
                c=MotionController(profile,BINDINGS,lambda p,m:True,tight_route())
                t=Telemetry(1,1,0,1,0,0,0,0,Flags.VALID,10,z=0,ground_speed=7)
                first=c.update(t,10)
                self.assertEqual(first.kind,'window',first.reason)
                self.assertEqual(first.event_id,0)
                self.assertFalse(c.slow_requested)
                self.assertEqual(sum(f.duration_ms for f in first.frames),200)
                self.assertTrue(all(84 not in f.keys and any(k in f.keys for k in (26,20,8)) for f in first.frames))
                pending=c.update(replace(t,seq=2,captured_at=10.033),10.033,first.frames[0].keys)
                self.assertEqual(pending.frames,first.frames)
                walking=c.update(replace(t,seq=3,captured_at=10.08,x=.1,speed=2.5),10.08,first.frames[0].keys)
                self.assertEqual(walking.kind,'window',walking.reason)
                self.assertEqual(walking.event_id,0)
                self.assertTrue(all(84 not in f.keys for f in walking.frames))
                self.assertFalse(c.slow_requested)

    def test_probe_only_toggles_after_live_running_speed_is_observed(self):
        c=MotionController(PROFILE,BINDINGS,lambda p,m:True,tight_route())
        t=Telemetry(1,1,0,1,0,0,0,0,Flags.VALID,10,z=0,ground_speed=7)
        probe=c.update(t,10)
        fast=replace(t,seq=2,captured_at=10.08,speed=7)
        toggle=c.update(fast,10.08,probe.frames[0].keys)
        self.assertEqual(toggle.kind,'window',toggle.reason)
        self.assertGreater(toggle.event_id,0)
        self.assertIn(84,toggle.frames[0].keys)
        retry=c.update(replace(fast,seq=3,captured_at=10.12),10.12,probe.frames[0].keys)
        self.assertEqual(retry.event_id,toggle.event_id)
        accepted=c.update(replace(fast,seq=4,captured_at=10.16),10.16,toggle.frames[-1].keys,toggle.event_id)
        self.assertEqual(accepted.event_id,0)
        self.assertTrue(all(84 not in f.keys for f in accepted.frames))

    def test_stationary_probe_still_requires_full_collision_certificate(self):
        c=MotionController(PROFILE,BINDINGS,lambda p,m:False,tight_route())
        t=Telemetry(1,1,0,1,0,0,0,0,Flags.VALID,10,z=0,ground_speed=7)
        result=c.update(t,10)
        self.assertEqual(result.kind,'stop')
        self.assertFalse(result.frames)
        self.assertFalse(c.probe_keys)

    def test_walk_ack_waits_without_extending_uncertified_future_motion(self):
        for applied in (False,True):
            with self.subTest(applied=applied):
                c=MotionController(replace(PROFILE,latency=.1),BINDINGS,lambda p,m:True,tight_route())
                t=Telemetry(1,1,0,1,0,0,0,7,Flags.VALID,10,z=0,ground_speed=7)
                first=c.update(t,10)
                self.assertGreater(first.event_id,0)
                c.validate=lambda p,m:False
                waiting=c.update(replace(t,seq=2,captured_at=10.04),10.04,first.frames[-1].keys,
                    first.event_id if applied else 0)
                self.assertEqual(waiting.kind,'wait' if applied else 'stop')
                self.assertFalse(waiting.frames)
                if applied:
                    deadline=c.slow_deadline
                    again=c.update(replace(t,seq=3,captured_at=10.08),10.08,first.frames[-1].keys,first.event_id)
                    self.assertEqual(again.kind,'wait')
                    self.assertEqual(c.slow_deadline,deadline)
                    expired=c.update(replace(t,seq=4,captured_at=deadline+.001),deadline+.001,first.frames[-1].keys,first.event_id)
                    self.assertEqual(expired.kind,'stop')

    def test_walk_probe_prediction_includes_old_keys_during_input_latency(self):
        traces=[]
        profile=replace(PROFILE,latency=.1)
        c=MotionController(profile,BINDINGS,lambda p,m:traces.append(p) or True,route((0,0),(20,0)))
        c.slow_requested=True;c.slow_origin=7;c.event_id=1;c.slow_keys=(26,)
        c.last_keys=(8,)  # Right strafe continues until new forward input arrives.
        t=Telemetry(1,1,0,1,0,0,0,7,Flags.VALID,10,z=0,ground_speed=7)
        result=c._walk_window(t,10,7,0,1)
        self.assertEqual(result.kind,'window')
        self.assertEqual(result.event_id,0)
        self.assertEqual(traces[-1][0],t.point)
        self.assertAlmostEqual(traces[-1][5].x,0)
        self.assertAlmostEqual(traces[-1][5].y,-.7)
        self.assertAlmostEqual(traces[-1][-1].x,1.4)
        self.assertAlmostEqual(traces[-1][-1].y,-.7)

    def test_walk_timeout_reports_actual_key_and_unconfirmed_speed(self):
        for key,label in ((84,'NUMPAD_DIVIDE'),(48,']')):
            with self.subTest(key=label):
                c=MotionController(PROFILE,(*BINDINGS[:5],key),lambda p,m:True,tight_route())
                t=Telemetry(1,1,0,1,0,0,0,7,Flags.VALID,10,z=0,ground_speed=7)
                first=c.update(t,10)
                c.update(replace(t,seq=2,captured_at=10.04),10.04,first.frames[-1].keys,first.event_id)
                deadline=c.slow_deadline
                result=c.update(replace(t,seq=3,captured_at=deadline+.001),deadline+.001,
                    first.frames[-1].keys,first.event_id)
                self.assertEqual(result.kind,'stop')
                self.assertEqual(result.reason_code,'motion_walk_unconfirmed')
                self.assertIn(f'走跑键 {label}',result.reason)
                self.assertIn('未观测到有效降速',result.reason)
                self.assertIn('切换前 7 码/秒，当前 7 码/秒',result.reason)
                self.assertNotIn('确认后的速度',result.reason)
                self.assertFalse(result.frames)
                self.assertEqual(c.slow_deadline,deadline)

    def test_walk_timeout_keeps_real_geometry_failure_after_confirmed_reduction(self):
        c=MotionController(PROFILE,BINDINGS,lambda p,m:True,tight_route())
        t=Telemetry(1,1,0,1,0,0,0,7,Flags.VALID,10,z=0,ground_speed=7)
        first=c.update(t,10)
        c.update(replace(t,seq=2,captured_at=10.04,x=.28),10.04,(26,),first.event_id)
        deadline=c.slow_deadline
        walking=c.update(replace(t,seq=3,captured_at=10.08,x=.38,speed=2.5),10.08,(26,),first.event_id)
        self.assertEqual(walking.kind,'window',walking.reason)
        self.assertTrue(c.slow_speed_confirmed)
        result=c.update(replace(t,seq=4,captured_at=deadline+.001,x=.38),deadline+.001,(26,),first.event_id)
        self.assertEqual(result.kind,'stop')
        self.assertEqual(result.reason_code,'motion_tight')
        self.assertIn('连续弯道空间不足',result.reason)
        self.assertNotIn('走跑',result.reason)
        self.assertFalse(result.frames)
        self.assertEqual(c.slow_deadline,deadline)

    def test_stopped_speed_is_not_confirmation_of_walk_mode(self):
        c=MotionController(PROFILE,BINDINGS,lambda p,m:True,tight_route())
        t=Telemetry(1,1,0,1,0,0,0,7,Flags.VALID,10,z=0,ground_speed=7)
        first=c.update(t,10)
        c.update(replace(t,seq=2,captured_at=10.04,speed=0),10.04,first.frames[-1].keys,first.event_id)
        self.assertFalse(c.slow_speed_confirmed)
        deadline=c.slow_deadline
        result=c.update(replace(t,seq=3,captured_at=deadline+.001,speed=0),deadline+.001,
            first.frames[-1].keys,first.event_id)
        self.assertEqual(result.kind,'stop')
        self.assertEqual(result.reason_code,'motion_walk_unconfirmed')
        self.assertIn('当前 0 码/秒',result.reason)

    def test_no_input_is_not_reported_as_a_stuck_character(self):
        c=MotionController(PROFILE,BINDINGS,lambda p,m:True,route((0,0),(60,0)))
        t=Telemetry(1,1,0,1,0,0,0,0,Flags.VALID,10,z=0,ground_speed=7)
        self.assertEqual(c.update(t,10).kind,'window')
        self.assertEqual(c.update(replace(t,seq=2,captured_at=13),13).kind,'window')

    def test_motion_failure_cannot_blacklist_the_players_entire_start_polygon(self):
        a,b=Point(0,0,0,0),Point(0,20,0,0)
        for edge,expected in (('nav:pack:0:11,20,5954:11,20,5954',None),
                              ('nav:pack:0:11,20,5954:11,20,5955','nav:pack:0:11,20,5954:11,20,5955')):
            r=Route(a,b,(Segment((a,b),'fixture','navmesh',edge),))
            c=MotionController(PROFILE,BINDINGS,lambda p,m:True,r)
            t=Telemetry(1,1,0,1,0,0,0,0,Flags.VALID,10,z=0,ground_speed=7)
            self.assertEqual(c.reply(t,'replan','fixture',block=True).blocked_edge,expected)

    def test_slope_uses_surface_distance_for_horizontal_speed(self):
        a,b=Point(0,0,0,0),Point(0,20,0,15)
        c=MotionController(PROFILE,BINDINGS,lambda p,m:True,Route(a,b,(Segment((a,b),'slope','navmesh','slope'),)))
        t=Telemetry(1,1,0,1,0,0,0,10,Flags.VALID,10,z=0,ground_speed=10)
        trace=[]
        c.validate=lambda p,m:trace.append(p) or True
        w=c.update(t,10)
        self.assertEqual(w.kind,'window')
        self.assertAlmostEqual(trace[-1][-1].x,1.6,places=5)
        self.assertAlmostEqual(trace[-1][-1].z,1.2,places=5)

    def test_stop_prediction_accounts_for_latency_without_claiming_arrival(self):
        c=MotionController(replace(PROFILE,latency=.2),BINDINGS,lambda p,m:True,route((0,0),(20,0)))
        t=Telemetry(1,1,0,1,0,0,0,7,Flags.VALID,10,z=0,ground_speed=7)
        self.assertEqual(c.update(t,10).kind,'window')
        approaching=replace(t,seq=2,captured_at=12.65,x=18.6)
        result=c.update(approaching,12.65,(26,))
        self.assertEqual(result.kind,'settle');self.assertFalse(result.frames)


if __name__=='__main__':unittest.main()
