"""Pure, ground motion geometry and time-parameterized keyboard trajectories.

Every shortcut, curve and predicted input trace is checked by the supplied
surface validator. No screen, device, networking or inferred game altitude.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

from .models import Point, Route, Telemetry, PlanningError, angle_error
from .controller import Calibration, unsafe_reason
from .motion_protocol import TimedKeyframe
from .settings import key_name

CURVE_ERROR = .01
TRACK_MARGIN = .10
MAX_SOURCE_AGE = .08


def uncertainty_margin(speed):
    # XY telemetry rounds to 1/16 yd (radial <= .0442). Include dispatch age,
    # one telemetry period of input-latency uncertainty, USB jitter and model
    # error across the short execution horizon. This is EXTRA to baked radius.
    return TRACK_MARGIN + speed*(MAX_SOURCE_AGE+.034+.005+.04*.2)


def interpolate(a, b, u):
    return Point(a.c,a.x+(b.x-a.x)*u,a.y+(b.y-a.y)*u,
                 None if a.z is None or b.z is None else a.z+(b.z-a.z)*u)


@dataclass(frozen=True)
class MotionProfile:
    left_rate: float
    right_rate: float
    latency: float = 0.
    strafe_left_angle: float = math.pi/2
    strafe_right_angle: float = -math.pi/2
    diagonal_left_angle: float = math.pi/4
    diagonal_right_angle: float = -math.pi/4
    strafe_verified: bool = False
    walk_ratio: float | None = None
    walk_toggle_allowed: bool = True
    version: int = 1

    @classmethod
    def from_calibration(cls, calibration, movement=None):
        return cls(calibration.left_rate_rad_s,calibration.right_rate_rad_s,
                   calibration.input_latency_s,**(movement or {}))


@dataclass(frozen=True)
class MotionPrimitive:
    start: Point
    end: Point
    center: Point | None = None
    sweep: float = 0.

    @property
    def radius(self):
        return self.start.distance(self.center) if self.center else math.inf

    @property
    def length(self):
        return abs(self.sweep)*self.radius if self.center else self.start.distance(self.end)

    @property
    def length_3d(self):
        return math.hypot(self.length,(self.end.z or 0)-(self.start.z or 0))

    def point(self,u):
        if self.center is None:return interpolate(self.start,self.end,u)
        a=math.atan2(self.start.y-self.center.y,self.start.x-self.center.x)+self.sweep*u
        z=interpolate(self.start,self.end,u).z
        return Point(self.start.c,self.center.x+self.radius*math.cos(a),self.center.y+self.radius*math.sin(a),z)

    def heading(self,u):
        if self.center is None:return math.atan2(self.end.y-self.start.y,self.end.x-self.start.x)
        return math.atan2(self.start.y-self.center.y,self.start.x-self.center.x)+self.sweep*u+math.copysign(math.pi/2,self.sweep)

    def sample(self):
        n=1 if self.center is None else max(1,math.ceil(abs(self.sweep)/
            (2*math.acos(max(-1.,1-CURVE_ERROR/self.radius)))))
        return tuple(self.point(i/n) for i in range(n+1))


@dataclass(frozen=True)
class MotionTrajectory:
    primitives: tuple[MotionPrimitive, ...]
    source_points: tuple[Point, ...]
    speed: float
    model_version: int
    # Ordered passage thresholds for EVERY original source point. A rounded
    # corner is passed at its arc exit, not by reaching the removed vertex.
    source_progress: tuple[float, ...] = ()

    @property
    def points(self):
        return tuple(p for i,s in enumerate(self.primitives) for p in s.sample()[int(i>0):])

    @property
    def length(self):return sum(s.length for s in self.primitives)

    def at(self,distance):
        for s in self.primitives:
            if distance<=s.length:return s.point(max(0.,distance)/max(s.length,1e-9)),s.heading(max(0.,distance)/max(s.length,1e-9)),s
            distance-=s.length
        s=self.primitives[-1];return s.end,s.heading(1),s

    def project(self,p,progress,span):
        # Forward-local progress, never the globally nearest point of a loop.
        best=None; offset=0.
        for segment in self.primitives:
            end=offset+segment.length
            if end<progress-.35:offset=end;continue
            if offset>progress+span:break
            if segment.center is None:
                a,b=segment.start,segment.end
                u=((p.x-a.x)*(b.x-a.x)+(p.y-a.y)*(b.y-a.y))/max(segment.length**2,1e-12)
            else:
                a=math.atan2(p.y-segment.center.y,p.x-segment.center.x)
                initial=math.atan2(segment.start.y-segment.center.y,segment.start.x-segment.center.x)
                u=angle_error(a,initial)/segment.sweep
            u=max(0.,min(1.,u));q=segment.point(u);s=offset+u*segment.length
            if s>=progress-.35 and (best is None or p.distance(q)<best[0]):best=(p.distance(q),s,q)
            offset=end
        return best


def smooth_route(points,speed,profile,validate):
    points=tuple(points)
    if len(points)<2:raise PlanningError('motion_empty','没有剩余运动路线')
    if any(p.z is None for p in points):raise PlanningError('motion_height','连续运动需要确定的地面')
    # Merge short straight/jitter sections only within a bounded source arc.
    original=points;simplified=[points[0]];source_indices=[0];i=0
    while i<len(points)-1:
        chosen=i+1;distance=0.
        for j in range(i+1,min(len(points),i+33)):
            distance+=points[j-1].distance(points[j]);chord=points[i].distance(points[j])
            # A portal may split one straight road into arbitrarily short
            # pieces. Do not reserve a separate bend radius for each piece.
            if distance>12 and distance>chord*1.0001+.001:break
            dx,dy=points[j].x-points[i].x,points[j].y-points[i].y
            monotone=all((b.x-a.x)*dx+(b.y-a.y)*dy>=-1e-6 for a,b in zip(points[i:j],points[i+1:j+1]))
            if distance<=chord*1.5+.03 and monotone and validate((points[i],points[j]),TRACK_MARGIN):chosen=j
        simplified.append(points[chosen]);source_indices.append(chosen);i=chosen
    points=tuple(simplified);primitives=[];cursor=points[0];corners={}
    for i in range(1,len(points)-1):
        a,b,c=points[i-1:i+2];first=a.distance(b);second=b.distance(c)
        if min(first,second)<1e-6:continue
        h0=math.atan2(b.y-a.y,b.x-a.x);h1=math.atan2(c.y-b.y,c.x-b.x);turn=angle_error(h1,h0)
        if abs(turn)<math.radians(.5):continue
        if abs(turn)>math.radians(170):raise PlanningError('motion_tight','回转空间不足，需要道路内绕行')
        # Preserve steering authority for feedback rather than saturating every bend.
        omega=profile.left_rate if turn>0 else profile.right_rate
        horizontal=max(first/math.hypot(first,b.z-a.z),second/math.hypot(second,c.z-b.z))
        radius=speed*horizontal/omega*1.15
        trim=radius*math.tan(abs(turn)/2)
        corners[i]=(radius,trim,turn,h0)
    # Adjacent fillets share a segment. Allocate their actual tangent lengths
    # instead of arbitrarily limiting every isolated corner to half a segment.
    for i in range(len(points)-1):
        used=corners.get(i,(0,0,0,0))[1]+corners.get(i+1,(0,0,0,0))[1]
        if used>points[i].distance(points[i+1])-1e-6:
            raise PlanningError('motion_tight','当前速度下连续弯道空间不足，需要走路或道路内绕行')
    source_progress=[0.]*len(original);previous_source=0;length=0.
    for i,(radius,trim,turn,h0) in corners.items():
        a,b,c=points[i-1:i+2];first=a.distance(b);second=b.distance(c)
        entry=interpolate(b,a,trim/first);exit=interpolate(b,c,trim/second)
        sign=1 if turn>0 else -1
        center=Point(b.c,entry.x-sign*math.sin(h0)*radius,entry.y+sign*math.cos(h0)*radius,entry.z)
        straight_length=cursor.distance(entry);straight_start=length
        if straight_length>1e-6:
            primitives.append(MotionPrimitive(cursor,entry));length+=straight_length
        arc=MotionPrimitive(entry,exit,center,turn);primitives.append(arc);length+=arc.length
        # Map merged portal/jitter points in source order onto the actual
        # straight primitive. Its incoming trim and the removed corner vertex
        # belong to the arc exit; the previous outgoing trim is already passed.
        for j in range(previous_source+1,source_indices[i]+1):
            p=original[j]
            along=(((p.x-cursor.x)*(entry.x-cursor.x)+(p.y-cursor.y)*(entry.y-cursor.y))/straight_length
                   if straight_length>1e-6 else (p.x-cursor.x)*math.cos(h0)+(p.y-cursor.y)*math.sin(h0))
            passed=length if j==source_indices[i] or along>=straight_length-1e-6 else straight_start+max(0.,along)
            source_progress[j]=max(source_progress[j-1],passed)
        previous_source=source_indices[i];cursor=exit
    straight_length=cursor.distance(points[-1]);heading=math.atan2(points[-1].y-cursor.y,points[-1].x-cursor.x)
    if straight_length>1e-6:primitives.append(MotionPrimitive(cursor,points[-1]))
    for j in range(previous_source+1,len(original)):
        p=original[j];along=(p.x-cursor.x)*math.cos(heading)+(p.y-cursor.y)*math.sin(heading)
        source_progress[j]=max(source_progress[j-1],length+max(0.,min(straight_length,along)))
    for segment in primitives:
        if not validate(segment.sample(),TRACK_MARGIN+(CURVE_ERROR if segment.center else 0)):
            raise PlanningError('motion_collision','连续轨迹与地面边界或楼层不兼容')
    return MotionTrajectory(tuple(primitives),original,speed,profile.version,tuple(source_progress))


def join_trajectory(trajectory,t,speed,profile,validate):
    """Join the first straight via certified circle/line/circle paths.

    Character facing and travel heading differ when strafing. A fixed travel
    offset supplies each candidate's start tangent; the normal key controller
    follows the resulting curve while moving. Never skip the first real bend,
    a later loop leg, or an unverified floor to align the character.
    """
    first=trajectory.primitives[0];heading=first.heading(0)
    offsets=(0.,profile.strafe_left_angle,profile.strafe_right_angle,
             profile.diagonal_left_angle,profile.diagonal_right_angle) if profile.strafe_verified else (0.,)
    if min(abs(angle_error(heading,t.facing+o)) for o in offsets)<=math.radians(20):
        return trajectory,0.
    if first.center is not None:
        raise PlanningError('motion_join','当前朝向无法连续接入首个弯道，请在道路开阔处起步')
    radius=speed/min(profile.left_rate,profile.right_rate)*1.15
    candidates=[]
    for distance in sorted({min(first.length,d) for d in (max(4.,radius*2),max(8.,radius*3),max(12.,radius*4))}):
        if distance<1:continue
        target=first.point(distance/first.length)
        for offset in offsets:
            initial=t.facing+offset
            for s0 in (-1,1):
                for s1 in (-1,1):
                    c0=Point(t.c,t.x-s0*radius*math.sin(initial),t.y+s0*radius*math.cos(initial),t.z)
                    c1=Point(t.c,target.x-s1*radius*math.sin(heading),target.y+s1*radius*math.cos(heading),target.z)
                    separation=c0.distance(c1)
                    if separation<1e-6 or abs((s1-s0)*radius)>separation:continue
                    tangent=math.atan2(c1.y-c0.y,c1.x-c0.x)-math.asin((s1-s0)*radius/separation)
                    a=Point(t.c,c0.x+s0*radius*math.sin(tangent),c0.y-s0*radius*math.cos(tangent),t.z)
                    b=Point(t.c,c1.x+s1*radius*math.sin(tangent),c1.y-s1*radius*math.cos(tangent),target.z)
                    turn0=s0*((s0*(tangent-initial))%math.tau)
                    turn1=s1*((s1*(heading-tangent))%math.tau)
                    # Do not make a nearly full circle just to align at a close
                    # goal. Such a start needs a different certified corridor.
                    if max(abs(turn0),abs(turn1))>math.pi:continue
                    raw=[]
                    if abs(turn0)>1e-6:raw.append(MotionPrimitive(t.point,a,c0,turn0))
                    if a.distance(b)>1e-6:raw.append(MotionPrimitive(a,b))
                    if abs(turn1)>1e-6:raw.append(MotionPrimitive(b,target,c1,turn1))
                    length=sum(p.length for p in raw)
                    if not raw or length>max(12.,distance*2.5):continue
                    # Reference height follows the same source floor; the
                    # validator resolves terrain detail and checks the sweep.
                    prefix=[];along=0.
                    for primitive in raw:
                        start=replace(primitive.start,z=t.z+(target.z-t.z)*along/length)
                        along+=primitive.length
                        end=replace(primitive.end,z=t.z+(target.z-t.z)*along/length)
                        prefix.append(replace(primitive,start=start,end=end))
                    tail=(() if first.length-distance<1e-6 else (replace(first,start=target),))+trajectory.primitives[1:]
                    candidates.append((length-distance,tuple(prefix),tail,length,distance))
    for _,prefix,tail,length,distance in sorted(candidates,key=lambda c:c[0]):
        points=tuple(p for i,primitive in enumerate(prefix) for p in primitive.sample()[int(i>0):])
        if validate(points,uncertainty_margin(speed)+CURVE_ERROR):
            # Source points consumed by the join are passed only at its exit.
            # The observed start itself remains at zero; later thresholds shift
            # by the added prefix length without searching nearby loop legs.
            progress=tuple(0. if i==0 else length if s<=distance else s+length-distance
                           for i,s in enumerate(trajectory.source_progress))
            return replace(trajectory,primitives=prefix+tail,source_progress=progress),length
    raise PlanningError('motion_join','当前朝向没有通过碰撞验证的连续起步曲线，请在道路开阔处起步')


@dataclass(frozen=True)
class MotionWindow:
    frames: tuple[TimedKeyframe, ...]
    source: Telemetry
    route_version: int
    model_version: int
    trajectory: tuple[Point, ...]
    predicted_start: Point
    event_id: int = 0
    kind: str = 'window'
    reason: str = ''
    actual_keys: tuple[int,...] = ()
    margin: float = 0.
    blocked_edge: str | None = None
    reason_code: str = ''


def propagate(p,facing,speed,keys,dt,profile,bindings):
    forward,left,right,strafe_left,strafe_right=bindings[:5]
    yaw=(profile.left_rate if left in keys else 0)-(profile.right_rate if right in keys else 0)
    direction=0.
    if strafe_left in keys:direction=profile.diagonal_left_angle if forward in keys else profile.strafe_left_angle
    if strafe_right in keys:direction=profile.diagonal_right_angle if forward in keys else profile.strafe_right_angle
    moving=any(k in keys for k in (forward,strafe_left,strafe_right))
    speed=speed if moving else 0.;heading=facing+direction
    if abs(yaw)<1e-8:dx,dy=speed*dt*math.cos(heading),speed*dt*math.sin(heading)
    else:
        dx=speed/yaw*(math.sin(heading+yaw*dt)-math.sin(heading))
        dy=speed/yaw*(math.cos(heading)-math.cos(heading+yaw*dt))
    return Point(p.c,p.x+dx,p.y+dy,p.z),(facing+yaw*dt)%math.tau


class MotionController:
    """Persistent, pure controller owned by the isolated motion worker."""
    def __init__(self,profile,bindings,validate,route,route_version=1):
        self.profile,self.bindings,self.validate=profile,tuple(bindings),validate
        self.route,self.route_version=route,route_version
        self.progress=0.;self.trajectory=None;self.last=None;self.last_keys=()
        self.last_speed=None;self.episode_start=None;self.episode_position=None
        self.slow_requested=False;self.slow_origin=None;self.event_id=0
        self.reference_index=0;self.preview_sent=False
        self.local_final=False;self.slow_deadline=0.
        self.observation=None;self.speed_mismatches=0
        self.joining_until=0.
        self.source_start_index=0
        self.join_base=None
        self.slow_keys=();self.slow_applied=False
        self.slow_speed_confirmed=False
        self.probe_keys=()

    def _validate_prediction(self,points,margin):
        # The first point is observed. Later Z values are predictions along
        # the reference surface and may cross a ground crest between anchors.
        validate=getattr(self.validate,'validate_prediction',self.validate)
        return validate(points,margin)

    def _walk_window(self,t,now,speed,heading,applied_event,geometry_error=None):
        if self.slow_requested and applied_event==self.event_id and not self.slow_applied:
            self.slow_applied=True;self.slow_deadline=now+self.profile.latency+.25
        if self.slow_applied and now>=self.slow_deadline:
            if self.slow_speed_confirmed:
                # The game already demonstrated a lower moving speed. A
                # later infeasible curve is the current geometry failure,
                # not evidence that the earlier toggle never took effect.
                return self.reply(t,'stop',str(geometry_error),code=geometry_error.code) if geometry_error else None
            key=key_name(self.bindings[5])
            return self.reply(t,'stop',f'走跑键 {key} 已由 USB 接受，但未观测到有效降速：切换前 {self.slow_origin:g} 码/秒，当前 {t.speed:g} 码/秒；请核对游戏内走跑绑定',code='motion_walk_unconfirmed')
        forward,left,right,sl,sr=self.bindings[:5]
        bases=[(forward,)]
        if self.profile.strafe_verified:bases.extend(((sl,),(sr,),(forward,sl),(forward,sr)))
        candidates=[self.slow_keys] if self.slow_requested else [self.probe_keys] if self.probe_keys and t.speed<=.05 else [
            tuple(sorted((*base,*(() if turn is None else (turn,)))))
            for base in bases for turn in (None,left,right)]
        safe=[]
        for keys in candidates:
            p=t.point;facing=t.facing;trace=[p]
            delay=self.profile.latency+max(0.,now-t.captured_at)
            # The acknowledged old keys remain effective until the new input
            # arrives. Certify that prefix too, rather than applying a newly
            # selected probe/turn throughout the latency interval.
            for held,seconds in ((self.last_keys,delay),(keys,.2)):
                steps=max(1,math.ceil(seconds/.02))
                for _ in range(steps):
                    p,facing=propagate(p,facing,speed,held,seconds/steps,self.profile,self.bindings)
                    trace.append(p)
            direction=math.atan2(p.y-t.y,p.x-t.x)
            score=abs(angle_error(heading,direction))
            safe.append((score,keys,tuple(trace)))
        for _,keys,trace in sorted(safe,key=lambda v:v[0]):
            if not self._validate_prediction(trace,uncertainty_margin(speed)+CURVE_ERROR):continue
            if not self.slow_requested and t.speed<=.05:
                # Zero current speed cannot reveal whether the game is already
                # walking. Move within the certified upper-bound trace first;
                # the next live speed decides whether a toggle is necessary.
                self.probe_keys=keys
                return MotionWindow((TimedKeyframe(200,keys),),t,self.route_version,self.profile.version,(),t.point,
                    reason='连续起步测量实际速度；确认后决定是否降速',margin=uncertainty_margin(speed)+CURVE_ERROR)
            if not self.slow_requested:
                if not self.profile.walk_toggle_allowed:return None
                self.slow_requested=True;self.slow_origin=speed;self.event_id+=1;self.slow_keys=keys
            # A computed one-shot is not yet a submitted one-shot. Retry the
            # SAME id after stale worker results until USB acceptance is seen.
            event=0 if self.slow_applied else self.event_id
            frames=(TimedKeyframe(20,tuple(sorted((*keys,self.bindings[5])))),TimedKeyframe(180,keys)) if event else (TimedKeyframe(200,keys),)
            return MotionWindow(frames,t,self.route_version,self.profile.version,(),t.point,event,
                reason='连续移动中切换走路，等待实时速度确认',margin=uncertainty_margin(speed)+CURVE_ERROR)
        if self.slow_applied:
            # No newly certified extension is available. Retain only the
            # previously accepted window until fresh telemetry confirms the
            # game's speed change. A wait neither replaces nor extends it.
            return self.reply(t,'wait','走跑切换已提交，等待实时速度确认',code='motion_walk_pending')
        return None

    def reply(self,t,kind,reason='',block=False,code=''):
        edge=None
        if block:
            # Use the local original route's identity, never the globally nearest
            # leg of a loop. Replanning can search another verified road corridor.
            index=0
            for segment in self.route.segments:
                if index+len(segment.points)-1>self.reference_index:
                    candidate=segment.edge_id
                    # A failed motion fit is not evidence that the entire
                    # polygon containing the player is impassable.
                    if not candidate.startswith('nav:') or candidate.split(':')[-2]!=candidate.split(':')[-1]:
                        edge=candidate
                    break
                index+=len(segment.points)-1
        return MotionWindow((),t,self.route_version,self.profile.version,(),t.point,kind=kind,reason=reason,blocked_edge=edge,reason_code=code)

    def update(self,t,now,actual_keys=(),applied_event=0):
        reason=unsafe_reason(t,now)
        if reason:return self.reply(t,'stop',reason)
        if t.protocol_version!=4:return self.reply(t,'stop','请更新 v4 速度遥测插件')
        if t.environment&8:return self.reply(t,'stop','角色离开地面；本轮不支持跳跃')
        if t.z is None:return self.reply(t,'stop','当前楼层未确定')
        if self.last and (t.session!=self.last.session or t.c!=self.last.c):return self.reply(t,'stop','遥测会话或大陆改变')
        if t.c!=self.route.start.c:return self.reply(t,'stop','路线大陆不符')
        if self.last and t.seq==self.last.seq:return self.reply(t,'wait')
        if t.speed<0 or not math.isfinite(t.speed):return self.reply(t,'stop','实时速度无效')
        if self.slow_requested and (self.slow_applied or applied_event==self.event_id) and .05<t.speed<self.slow_origin*.9:
            # Keep the observation even after normal movement succeeds or a
            # later speed increase makes another curve infeasible. Zero speed
            # is a stop, not proof that the game's walk mode changed.
            self.slow_speed_confirmed=True
        speed=t.speed if t.speed>.05 else t.ground_speed
        if speed is None or not math.isfinite(speed) or speed<=0:return self.reply(t,'stop','插件未提供有效的起步地面速度')
        # Model corrections use the effective speed NOW, including decreases.
        actual_keys=tuple(actual_keys)
        if self.last and actual_keys and actual_keys==self.last_keys and t.speed>.05:
            dt=t.captured_at-self.last.captured_at
            if .015<=dt<=.15:
                yaw=angle_error(t.facing,self.last.facing)/dt
                left,right=self.bindings[1:3]
                if left in actual_keys and .2<yaw<12 and abs(yaw-self.profile.left_rate)<self.profile.left_rate*.25:
                    self.profile=replace(self.profile,left_rate=.98*self.profile.left_rate+.02*yaw,version=self.profile.version+1)
                elif right in actual_keys and -12<yaw<-.2 and abs(-yaw-self.profile.right_rate)<self.profile.right_rate*.25:
                    self.profile=replace(self.profile,right_rate=.98*self.profile.right_rate-.02*yaw,version=self.profile.version+1)
        if self.observation and t.captured_at-self.observation.captured_at>=.25:
            old=self.observation;dt=t.captured_at-old.captured_at
            measured=math.hypot(old.point.distance(t.point),t.z-old.z)/dt
            expected=(old.speed+t.speed)/2
            self.speed_mismatches=self.speed_mismatches+1 if abs(measured-expected)>max(1.5,expected*.45) else 0
            if self.speed_mismatches>=2:return self.reply(t,'replan','实时速度与连续位置变化不符，可能受阻或被外力移动')
            self.observation=t
        elif self.observation is None:self.observation=t
        self.last_keys=actual_keys
        self.last=t
        moving=any(k in actual_keys for k in (self.bindings[0],*self.bindings[3:5]))
        if not moving:self.episode_start=None;self.episode_position=None
        elif self.episode_start is None:self.episode_start=t.captured_at;self.episode_position=t.point
        elif t.captured_at-self.episode_start>=2:
            if self.episode_position.distance(t.point)<.5:return self.reply(t,'replan','连续移动两秒但位移不足 0.5 码')
            self.episode_start=t.captured_at;self.episode_position=t.point
        original=self.route.points
        if self.trajectory:
            projected=self.trajectory.project(t.point,self.progress,max(3.,speed*.4))
            if not projected:return self.reply(t,'replan','无法沿当前轨迹确认进度')
            error,s,q=projected
            if error>.75 or abs(t.z-q.z)>.8:return self.reply(t,'replan','偏离连续运动走廊')
            self.progress=max(self.progress,s)
            # remaining[0] was the observed position at source_start_index;
            # remaining[j] is original[source_start_index+j]. Only certified
            # forward-local trajectory progress can advance that source order.
            while self.reference_index<len(original)-2:
                source=self.reference_index-self.source_start_index+1
                if source>=len(self.trajectory.source_progress) or self.progress+1e-6<self.trajectory.source_progress[source]:break
                self.reference_index+=1
            if self.joining_until and self.progress>=self.joining_until:
                self.joining_until=0.
            if self.local_final and t.speed<=.05 and self.trajectory.length-self.progress<=.8 and t.point.distance(self.route.goal)<=.55 and abs(t.z-self.route.goal.z)<=1:
                return self.reply(t,'arrived','已到达原始终点')
        # Refit only the remaining original source arc when speed materially changes.
        changed=self.trajectory is None or any(s.center and speed*s.length/max(s.length_3d,1e-9)>
            s.radius*(self.profile.left_rate if s.sweep>0 else self.profile.right_rate)*.96 for s in self.trajectory.primitives)
        refresh=self.trajectory is not None and not self.local_final and self.trajectory.length-self.progress<max(5.,speed*2)
        if self.trajectory is None or changed or refresh:
            end=self.reference_index+1;distance=t.point.distance(original[end])
            while end<len(original)-1 and distance<max(24.,speed*4):
                distance+=original[end].distance(original[end+1]);end+=1
            remaining=(t.point,*original[self.reference_index+1:end+1])
            fitted=False
            try:
                trajectory=smooth_route(remaining,speed,self.profile,self.validate)
                trajectory,joining_until=join_trajectory(trajectory,t,speed,self.profile,self.validate)
                fitted=True
            except PlanningError as exc:
                # Refreshing the preview is optional while the existing
                # certified curve still covers the complete input horizon.
                # A removed source vertex can sit beside the current fillet;
                # never steer back to it merely to extend that preview.
                reserve=speed*(.2+self.profile.latency+max(0.,now-t.captured_at))+.6
                keep_existing=(refresh and not changed and
                    self.trajectory.length-self.progress>reserve)
                if not keep_existing:
                    # A walk request is a one-shot input, never W duty cycling.
                    can_slow=(exc.code in ('motion_tight','motion_collision','motion_join') and len(self.bindings)>5
                        and t.ground_speed and ((t.speed<=.05 and not self.slow_requested) or
                            (self.profile.walk_toggle_allowed and (t.speed<=.05 or t.speed>=t.ground_speed*.8))))
                    if can_slow and (not self.slow_requested or speed>=self.slow_origin*.9):
                        target=remaining[min(2,len(remaining)-1)]
                        heading=math.atan2(target.y-t.y,target.x-t.x)
                        transition=self._walk_window(t,now,speed,heading,applied_event,exc)
                        if transition:return transition
                    # Kinematic failure is not a blocked topology certificate.
                    return self.reply(t,'stop',str(exc),code=exc.code)
            if fitted:
                self.local_final=end==len(original)-1
                self.trajectory=trajectory;self.progress=0.;self.last_speed=speed;self.preview_sent=False
                self.joining_until=joining_until
                self.join_base=None
                self.source_start_index=self.reference_index
                if joining_until:
                    forward,_,_,sl,sr=self.bindings[:5]
                    modes=(((forward,),0.),((sl,),self.profile.strafe_left_angle),((sr,),self.profile.strafe_right_angle),
                           ((forward,sl),self.profile.diagonal_left_angle),((forward,sr),self.profile.diagonal_right_angle))
                    self.join_base=min(modes,key=lambda mode:abs(angle_error(trajectory.primitives[0].heading(0),t.facing+mode[1])))[0]
        self.last_speed=speed
        # Position at input arrival, using the current acknowledged key state.
        predicted,facing=t.point,t.facing;latency_trace=[t.point]
        delay=self.profile.latency+max(0.,now-t.captured_at)
        delay_steps=max(1,math.ceil(delay/.02))
        for _ in range(delay_steps):
            primitive=self.trajectory.at(self.progress)[2]
            horizontal_speed=speed*primitive.length/max(primitive.length_3d,1e-9)
            predicted,facing=propagate(predicted,facing,horizontal_speed,actual_keys,delay/delay_steps,self.profile,self.bindings)
            projected=self.trajectory.project(predicted,self.progress,max(3.,speed*(self.profile.latency+.3)))
            if projected:predicted=replace(predicted,z=projected[2].z)
            latency_trace.append(predicted)
        result=self.trajectory.project(predicted,self.progress,max(3.,speed*(self.profile.latency+.3)))
        if not result:return self.reply(t,'replan','输入延迟后的预测位置超出路线')
        _,progress,_=result
        if self.local_final and (predicted.distance(self.route.goal)<=.30 or
                progress>=self.trajectory.length-.05):
            # Release before the input latency has carried the character past
            # the goal. The runtime must observe an actual stop and position;
            # this forecast is never reported as completed arrival.
            return self.reply(t,'settle','已开始停止，等待新遥测确认终点位置')
        forward,left,right,sl,sr=self.bindings[:5]
        frames=[];trace=latency_trace;p=predicted;previous_keys=tuple(actual_keys)
        for _ in range(10):
            target,heading,primitive=self.trajectory.at(min(self.trajectory.length,progress+max(.25,speed*.10)))
            horizontal_speed=speed*primitive.length/max(primitive.length_3d,1e-9)
            # Tangent feed-forward plus local lateral error, with a heading deadband.
            reference=self.trajectory.at(progress)[0]
            side=-(p.x-reference.x)*math.sin(heading)+(p.y-reference.y)*math.cos(heading)
            desired=heading-math.atan2(side,max(.5,speed*.20))
            error=angle_error(desired,facing)
            bases=[(forward,)]
            if self.profile.strafe_verified:
                bases.extend(((forward,sl),(forward,sr),(sl,),(sr,)))
            if self.joining_until and progress<self.joining_until and self.join_base:
                bases=[self.join_base]
            scored=[]
            for base in bases:
                for turn_key in (None,left,right):
                    keys=tuple(sorted((*base,*(() if turn_key is None else (turn_key,)))))
                    # Preserve a deadband on straight lines, but allow arc feed-forward.
                    q,h=propagate(p,facing,horizontal_speed,keys,.02,self.profile,self.bindings)
                    local=self.trajectory.project(q,progress,max(2.,speed*.15))
                    if not local:continue
                    cross,s,ground=local;q=replace(q,z=ground.z)
                    if s<progress-.03:continue
                    direction=math.atan2(q.y-p.y,q.x-p.x)
                    next_heading=self.trajectory.at(s)[1]
                    # Track a corrected travel heading, not a one-step race
                    # for zero cross-track error. Opposite strafe changes can
                    # look cheap over 20 ms but oscillate after game latency.
                    course=next_heading-math.atan2(side,max(.5,speed*.20))
                    score=max(0.,cross-.05)*4+abs(angle_error(course,direction))*1.2-((s-progress)/max(speed,1e-3))
                    previous_base=tuple(k for k in previous_keys if k in (forward,sl,sr))
                    score+=.08*(tuple(sorted(base))!=previous_base)+.002*(keys!=previous_keys)
                    if abs(error)<math.radians(1.5) and primitive.center is None and turn_key:score+=.02
                    scored.append((score,keys,q,h,s))
            if not scored:return self.reply(t,'replan','没有可执行的连续按键组合')
            _,keys,q,h,s=min(scored,key=lambda x:x[0])
            p,facing,progress=q,h,max(progress,s);trace.append(p);previous_keys=keys
            if frames and frames[-1].keys==keys:frames[-1]=replace(frames[-1],duration_ms=frames[-1].duration_ms+20)
            else:frames.append(TimedKeyframe(20,keys))
            if self.trajectory.length-progress<=.25:break
        # Each simulated 20-ms curved step has bounded sagitta; the .01 margin
        # is conservative for <=12 rad/s and speed admitted by telemetry.
        sagitta=max(CURVE_ERROR,speed/max(self.profile.left_rate,self.profile.right_rate)*
                     (1-math.cos(max(self.profile.left_rate,self.profile.right_rate)*.01)))
        margin=uncertainty_margin(speed)+sagitta
        if not self._validate_prediction(tuple(trace),margin):return self.reply(t,'stop','预测按键轨迹未通过整段碰撞检查',code='motion_input_collision')
        preview=self.trajectory.points if not self.preview_sent else ()
        self.preview_sent=True
        return MotionWindow(tuple(frames),t,self.route_version,self.profile.version,preview,predicted,actual_keys=actual_keys,margin=margin)
