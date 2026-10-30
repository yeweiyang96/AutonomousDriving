"""A persistent, cancellable process owns native motion geometry and prediction.

Only one tiny request may be in flight. The UI/control loop never waits for a
native query and never queues old observations behind newer ones.
"""
from dataclasses import replace
from collections import OrderedDict
import atexit
import json
import math
import multiprocessing as mp
import multiprocessing.util  # Register multiprocessing cleanup before ours.
from pathlib import Path
import threading
import time
import weakref

from .models import PlanningError, Point
from .motion import MotionController
from .navmesh import NavMeshPack, Mesh


_workers=weakref.WeakSet()


def _shutdown_workers():
    # Interpreter shutdown is outside the control loop. Finish our owners
    # before multiprocessing's own atexit handler touches the same handles.
    workers=tuple(_workers)
    for worker in workers:worker.close()
    for worker in workers:worker._finished.wait(2)


atexit.register(_shutdown_workers)


class SurfaceValidator:
    GROUND_STEP=.10
    MAX_GROUND_DEVIATION=.50
    MAX_GROUND_POINTS=2048
    MAX_HEIGHT_QUERIES=4096

    def __init__(self,manifest):
        self.manifest=Path(manifest);self.manifest_stamp=self.manifest.stat().st_mtime_ns
        self.pack=NavMeshPack(self.manifest);self.mesh=None;self.c=None
        if self.pack.native.motion_validate is None or self.pack.native.motion_corridor is None:
            raise PlanningError('navmesh_capability','请发布带连续运动校验接口的导航库')
        self.corridor=();self.corridor_points=()
        self._ground_cache=OrderedDict();self._height_cache=OrderedDict();self._height_queries=0

    def prepare(self,points):
        if not points:return
        self._height_queries=0
        if self.manifest.stat().st_mtime_ns!=self.manifest_stamp:
            raise PlanningError('navmesh_changed','导航包在运动过程中改变，请重新开始')
        if self.mesh is None or self.c!=points[0].c:
            self.close();self.c=points[0].c;self.mesh=Mesh(self.pack,self.c)
        keys=set()
        for p in points:keys.update(self.pack.around(p))
        # Bound long sessions' memory, retaining only the current local region.
        if self.mesh.bytes>96*1024*1024 and not keys.issubset(self.mesh.loaded):
            self.mesh.close();self.mesh=Mesh(self.pack,self.c)
            self.corridor=();self.corridor_points=()
            self._ground_cache.clear();self._height_cache.clear()
        self.mesh.load(keys)
        if points!=self.corridor_points:
            self.corridor=self.mesh.motion_corridor(points,4.)
            self.corridor_points=points
            self._ground_cache.clear()

    def __call__(self,points,margin):
        return self.grounded(points,margin) is not None

    def validate_prediction(self,points,margin=.10):
        """Certify predicted displacement on this corridor's ground surface.

        The first XYZ is an observation and remains an immutable anchor.
        Later heights, including the endpoint, are estimates; only a unique
        nearby ground surface may replace them before the same swept check.
        """
        points=tuple(points)
        if not self.corridor or any(p.c!=self.c for p in points):return False
        return self._grounded(points,margin,prediction=True) is not None

    def _load_trace(self,points):
        keys=set()
        for p in points:keys.update(self.pack.around(p,0))
        self.mesh.load(keys)

    def _ground_height(self,point):
        if point in self._height_cache:
            self._height_cache.move_to_end(point)
            return self._height_cache[point]
        if self._height_queries>=self.MAX_HEIGHT_QUERIES:return None
        self._height_queries+=1
        heights=tuple(h for h in self.mesh.heights(point)
                      if math.isfinite(h) and abs(h-point.z)<=self.MAX_GROUND_DEVIATION)
        self._height_cache[point]=heights
        if len(self._height_cache)>8192:self._height_cache.popitem(last=False)
        return heights

    def grounded(self,points,margin=.10):
        """Return a swept-certified ground trace, or None; never move its XY.

        End-to-end height interpolation can cut through a small ground crest.
        After the unchanged native fast path rejects it, recover only a unique
        nearby ground height, then certify the ENTIRE trace with the exact same
        floor corridor and margin. Samples alone never grant movement authority.
        """
        return self._grounded(points,margin)

    def _grounded(self,points,margin,prediction=False):
        points=tuple(points)
        if not points or any(p.z is None for p in points):return None
        # Source paths keep BOTH endpoints. Their rejected certificate must
        # never be reused for a prediction that may fit its final height.
        key=('prediction',points,margin) if prediction else (points,margin)
        if key in self._ground_cache:
            self._ground_cache.move_to_end(key)
            return self._ground_cache[key]
        self._load_trace(points)
        if self.mesh.motion_validate(points,margin,self.corridor):
            result=points
        else:
            result=self._ground_trace(points,margin,preserve_end=not prediction)
        # A per-request work limit is not evidence that this geometry is
        # invalid. Let a later fresh request retry instead of caching starvation.
        if result is not None or self._height_queries<self.MAX_HEIGHT_QUERIES:
            self._ground_cache[key]=result
            if len(self._ground_cache)>128:self._ground_cache.popitem(last=False)
        return result

    def _ground_trace(self,points,margin,preserve_end=True):
        reference=[points[0]]
        for a,b in zip(points,points[1:]):
            length=a.distance(b)
            if not math.isfinite(length):return None
            pieces=max(1,math.ceil(length/self.GROUND_STEP))
            if len(reference)+pieces>self.MAX_GROUND_POINTS:return None
            for i in range(1,pieces+1):
                u=i/pieces
                reference.append(Point(a.c,a.x+(b.x-a.x)*u,a.y+(b.y-a.y)*u,a.z+(b.z-a.z)*u))
        self._load_trace(reference)
        grounded=[]
        for i,p in enumerate(reference):
            heights=self._ground_height(p)
            if heights is None or len(heights)!=1:return None
            height=heights[0]
            if i==0 or (preserve_end and i==len(reference)-1):
                # Always preserve the observed start. Source paths also keep
                # their end; a prediction's end is generated, not observed.
                # Native swept validation preserves reached polygon identity.
                anchor=points[0] if i==0 else points[-1]
                if abs(height-anchor.z)>.15:return None
                grounded.append(anchor)
            else:grounded.append(replace(p,z=height))
        grounded=tuple(grounded)
        return grounded if self.mesh.motion_validate(grounded,margin,self.corridor) else None

    def close(self):
        if self.mesh is not None:self.mesh.close()
        self.mesh=None;self.corridor=();self.corridor_points=()
        self._ground_cache.clear();self._height_cache.clear();self._height_queries=0


def _serve(connection,route,profile,bindings,manifest,generation):
    validator=None
    try:
        validator=SurfaceValidator(manifest)
        controller=MotionController(profile,bindings,validator,route,generation)
        connection.send(('ready',validator.pack.identity,validator.pack.raw['library']['sha256']))
        while True:
            request=connection.recv()
            if request is None:break
            if request[0]=='probe':
                _,telemetry,radius=request
                validator.prepare((telemetry.point,))
                valid=validator.mesh.motion_validate((telemetry.point,),radius,())
                connection.send(('probe',telemetry,radius,valid));continue
            telemetry,keys,applied_event=request
            start=time.monotonic()
            try:
                original=route.points;i=controller.reference_index
                # Include original local topology plus the observed start.
                end=i+1;distance=telemetry.point.distance(original[end])
                speed=telemetry.speed or telemetry.ground_speed or 0
                while end<len(original)-1 and distance<max(24.,speed*4):
                    distance+=original[end].distance(original[end+1]);end+=1
                local=(telemetry.point,*original[i+1:end+1])
                # Current position may lie on the rounded bend, so corridor
                # authority comes from the original source arc, not a guessed
                # direct connection back to its next vertex.
                validator.prepare(tuple(original[max(0,i-1):end+1]))
                result=controller.update(telemetry,time.monotonic(),keys,applied_event)
                connection.send(('result',result,(time.monotonic()-start)*1000))
            except PlanningError as exc:
                connection.send(('result',controller.reply(telemetry,'replan',str(exc),code=exc.code),(time.monotonic()-start)*1000))
    except (EOFError,BrokenPipeError):pass
    except Exception as exc:
        try:connection.send(('error',f'{type(exc).__name__}: {exc}'))
        except (EOFError,BrokenPipeError):pass
    finally:
        if validator:validator.close()
        connection.close()


class MotionWorker:
    def __init__(self,route,profile,bindings,manifest,generation):
        self.expected_identity=None;self.connection=None;self.process=None
        self.pending=False;self._ready=False;self.last_frame=None;self.identity=None
        self._cancel=threading.Event();self._started=threading.Event();self._finished=threading.Event()
        self._io_lock=threading.Lock();self._startup_error=None
        # Windows spawn and even a large manifest read can exceed the input
        # acknowledgement deadline. Neither startup nor reaping belongs on
        # the control thread. This owner also reaps a cancelled late startup.
        self._owner=threading.Thread(target=self._lifecycle,
            args=(route,profile,bindings,manifest,generation),daemon=True,name='motion-worker-lifecycle')
        _workers.add(self)
        self._owner.start()

    @property
    def ready(self):
        return self._ready and not self._cancel.is_set()

    def _lifecycle(self,route,profile,bindings,manifest,generation):
        connection=child=process=None
        try:
            expected=json.loads(Path(manifest).read_bytes())
            if self._cancel.is_set():return
            identity=(expected['identity'],expected['library']['sha256'])
            context=mp.get_context('spawn');connection,child=context.Pipe()
            process=context.Process(target=_serve,args=(child,route,profile,bindings,str(manifest),generation),daemon=True)
            process.start();child.close();child=None
            with self._io_lock:
                if self._cancel.is_set():return
                self.expected_identity=identity;self.connection=connection;self.process=process
                self._started.set()
            self._cancel.wait()
        except Exception as exc:
            self._startup_error=exc
            self._started.set()
        finally:
            # No join or terminate occurs on the control thread. Checking pid
            # also handles failures before Process.start creates a child.
            try:
                if process is not None:
                    try:
                        if process.pid is not None:
                            if process.is_alive():process.terminate()
                            process.join(timeout=.5)
                            if process.is_alive():process.kill();process.join()
                    finally:process.close()
            finally:
                try:
                    with self._io_lock:
                        try:
                            if connection is not None:connection.close()
                        finally:
                            if child is not None:child.close()
                finally:self._finished.set()

    def submit(self,t,keys,applied_event=0):
        frame=t.session,t.seq
        if not self.ready or self.pending or self.last_frame==frame or not self._io_lock.acquire(False):return False
        try:
            if not self.ready:return False
            self.connection.send((t,tuple(keys),applied_event));self.pending=True;self.last_frame=frame
            return not self._cancel.is_set()
        finally:self._io_lock.release()

    def probe(self,t,radius):
        if not self.ready or self.pending or not self._io_lock.acquire(False):return False
        try:
            if not self.ready:return False
            self.connection.send(('probe',t,radius));self.pending=True
            return not self._cancel.is_set()
        finally:self._io_lock.release()

    def poll(self):
        if self._cancel.is_set() or not self._started.is_set():return None
        if self._startup_error is not None:
            raise PlanningError('motion_worker',f'连续运动进程启动失败：{self._startup_error}')
        if not self._io_lock.acquire(False):return None
        try:
            if self._cancel.is_set():return None
            result=None
            while self.connection.poll():
                message=self.connection.recv()
                if message[0]=='ready':
                    if tuple(message[1:])!=self.expected_identity:
                        raise PlanningError('navmesh_changed','运动进程与当前导航包不匹配')
                    self._ready=True;self.identity=message[1:]
                elif message[0]=='result':self.pending=False;result=message[1:]
                elif message[0]=='probe':self.pending=False;result=message
                else:raise PlanningError('motion_worker',message[1])
            if self._cancel.is_set():return None
            if not self.process.is_alive():raise PlanningError('motion_worker','连续运动进程已退出')
            return result
        except (EOFError,OSError):
            if self._cancel.is_set():return None
            raise PlanningError('motion_worker','连续运动进程通信已中断') from None
        finally:self._io_lock.release()

    def close(self):
        self._cancel.set();self._ready=False;self.pending=False
