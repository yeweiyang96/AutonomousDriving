#!/usr/bin/env python3
"""Child-process verification of bounded native search and the recorded route.

Synthetic checks exercise the new ABI; actual route checks retain the full
manifest, compare original and candidate Python, and never save last-plan.
No tile, manifest, controller, movement policy, or user input file is modified.
"""
from __future__ import annotations
from contextlib import contextmanager
import argparse
import ctypes as C
import hashlib
import importlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import traceback

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
CALLBACK = C.CFUNCTYPE(C.c_int)

def sha(data): return hashlib.sha256(data).hexdigest()
def require(value, message):
    if not value: raise ValueError(message)
def read_json(path): return json.loads(Path(path).read_text(encoding='utf-8-sig'))
def xyz(point): return [point.x, point.y, point.z]
def write_json(path, value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
def file_info(path):
    data=Path(path).read_bytes()
    return {'path':str(Path(path).resolve()),'bytes':len(data),'sha256':sha(data)}
def file_module(name, path):
    spec=importlib.util.spec_from_file_location(name,Path(path))
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    return module

def native_checks(args):
    helper=file_module('_bounds_legacy_validation_helper',HERE/'verify_surface_diagnostics.py')
    wrapper=helper.load_wrapper()
    candidate=helper.Legacy(args.dll,wrapper.CANCEL)
    baseline=helper.Legacy(args.baseline_dll,wrapper.CANCEL)
    function=candidate.dll.ad_path_bounded_v3
    ptr,ints,floats,refs=C.c_void_p,C.POINTER(C.c_int),C.POINTER(C.c_float),C.POINTER(C.c_uint64)
    function.argtypes=[ptr,floats,floats,C.c_int,C.c_float,wrapper.CANCEL,floats,refs,C.c_int,ints,ints]
    function.restype=C.c_int
    fixture=Path(args.fixture_dir).resolve(strict=True)
    info=read_json(fixture/'fixture-info.json');points=info['points']
    require(info.get('synthetic') is True and info.get('coordinate_order')=='north,west,height','Unexpected synthetic fixture')
    checks=[]
    def record(name,details):checks.append({'name':name,'passed':True,'details':details})
    def bounded(handle,a,b,limit=1000,budget=10000,capacity=4096,cancel=None):
        allocated=max(1,min(65535,capacity));output=(C.c_float*(allocated*3))();outrefs=(C.c_uint64*allocated)()
        count,expanded=C.c_int(-1),C.c_int(-1)
        callback=wrapper.CANCEL(cancel) if cancel is not None else wrapper.CANCEL()
        rc=function(handle,None if a is None else helper.Legacy.point(a),None if b is None else helper.Legacy.point(b),budget,limit,callback,output,outrefs,capacity,C.byref(count),C.byref(expanded))
        require(0<=count.value<=allocated,'Native output count is invalid')
        require(rc==0 or count.value==0,'Bounded query exposed a failed/partial output')
        return {'return_code':rc,'expanded':expanded.value,'count':count.value,'points':[[output[i*3+j]for j in range(3)]for i in range(count.value)],'refs':list(outrefs[:count.value]),'budget_reason':candidate.reason(handle) if handle else None}
    pairs=[('ground_chain_start','ground_chain_goal'),('ring_start','ring_goal'),('lower','lower'),('lower','upper'),('water_chain_start','water_chain_goal'),('offmesh_start','offmesh_goal'),('water','lower'),('multilevel','lower')]
    comparisons=[]
    with candidate.context(fixture,info) as handle,baseline.context(fixture,info) as original:
        for a,b in pairs:
            old=baseline.path(original,points[a],points[b]);legacy=candidate.path(handle,points[a],points[b]);new=bounded(handle,points[a],points[b])
            normalized=dict(new,return_code=5) if new['return_code']==10 else new
            require(old==legacy==normalized,'Large-domain compatibility failed for '+a+' -> '+b)
            comparisons.append({'start':a,'goal':b,'return_code':new['return_code'],'expanded':new['expanded']})
        record('large_domain_preserves_legacy_path_and_filter',comparisons)
        negative=bounded(handle,points['ring_start'],points['ring_goal'],limit=20)
        loose=bounded(handle,points['ring_start'],points['ring_goal'],limit=21)
        length=sum(math.dist(a[:2],b[:2])for a,b in zip(loose['points'],loose['points'][1:]))
        require(negative['return_code']==10 and negative['count']==0,'Tight ring domain should have no complete candidate')
        require(loose['return_code']==0 and length>21,'Fixture must demonstrate that a search domain is not a path-length guarantee')
        record('domain_is_scoped_and_final_length_guard_still_required',{'tight_domain':negative,'loose_domain_limit':21,'returned_length_xy':length,'final_policy_must_reject_over_limit':True})
        zero=bounded(handle,points['ground_chain_start'],points['ground_chain_goal'],budget=0)
        stopped=bounded(handle,points['ground_chain_start'],points['ground_chain_goal'],cancel=lambda:1)
        small=bounded(handle,points['ground_chain_start'],points['ground_chain_goal'],capacity=1)
        require(zero['return_code']==6 and zero['budget_reason']['reason']==1,'Expansion limit was misclassified')
        require(stopped['return_code']==7,'Cancellation was not preserved')
        require(small['return_code']==6 and small['budget_reason']['reason']==4,'Output capacity was misclassified')
        record('limits_cancel_and_failed_output',{'budget_zero':zero,'cancelled':stopped,'output_capacity_one':small})
        invalid=[{'a':None},{'b':None},{'a':[math.nan,5,0]},{'a':[5,5,math.inf]},{'limit':math.inf},{'limit':math.nan},{'limit':-1},{'limit':1},{'budget':-1},{'capacity':0},{'capacity':65536}]
        for changes in invalid:
            options={'a':points['ground_chain_start'],'b':points['ground_chain_goal'],**changes}
            require(bounded(handle,**options)['return_code']==1,'Invalid bounded ABI arguments were accepted: '+str(changes))
        require(bounded(None,points['ground_chain_start'],points['ground_chain_goal'])['return_code']==1,'Null context accepted')
        record('invalid_arguments_rejected',{'cases':len(invalid)+1})
        after=candidate.path(handle,points['ring_start'],points['ring_goal'])
        old=baseline.path(original,points['ring_start'],points['ring_goal'])
        require(after==old,'Bounded calls changed subsequent legacy behavior')
        record('legacy_behavior_restored_after_bounded_calls',{'same_path_points_refs_expansion_and_reason':True})
    diagnostic=wrapper.Diagnostics(candidate.dll)
    with candidate.context(fixture,info) as handle:
        def key(name):
            result=diagnostic.surfaces(handle,points[name],height_tolerance=.01)
            require(len(result['surfaces'])==1,'Unexpected block fixture surface')
            s=result['surfaces'][0]
            return (C.c_int*3)(s['tile_x'],s['tile_y'],s['poly_index'])
        a,m,b=key('ground_chain_start'),key('ground_chain_middle'),key('ground_chain_goal')
        require(candidate.dll.ad_block(handle,a,m)==0,'Could not install fixture edge exclusion')
        outcomes=[]
        for first,last in [('ground_chain_start','ground_chain_goal'),('ground_chain_goal','ground_chain_start')]:
            old=candidate.path(handle,points[first],points[last]);new=bounded(handle,points[first],points[last])
            require((5 if new['return_code']==10 else new['return_code'])==old['return_code'] and new['return_code'] in(8,10),'Blocked edge behavior changed')
            outcomes.append({'direction':first+' -> '+last,'return_code':new['return_code']})
        require(candidate.dll.ad_block(handle,b,b)==0,'Could not install self exclusion')
        self_result=bounded(handle,points['ground_chain_goal'],points['ground_chain_goal'])
        require(self_result['return_code']==8,'Self-blocked endpoint passed')
        record('symmetric_edge_and_self_exclusions_preserved',{'edges':outcomes,'self':self_result})
    return checks

def load_project(directory,name):
    directory=Path(directory).resolve(strict=True)
    directory=directory/'adrive' if (directory/'adrive').is_dir() else directory
    require((directory/'navmesh.py').is_file(),'Candidate Python directory lacks adrive/navmesh.py')
    spec=importlib.util.spec_from_file_location(name,directory/'__init__.py',submodule_search_locations=[str(directory)])
    package=importlib.util.module_from_spec(spec);sys.modules[name]=package;spec.loader.exec_module(package)
    modules={key:importlib.import_module(name+'.'+key) for key in('models','data','road_corridor','navmesh')}
    return modules

def parse_debug(message):
    if 'NAV_DEBUG ' not in message:return None
    result,_=json.JSONDecoder().raw_decode(message.split('NAV_DEBUG ',1)[1]);return result

def route_run(project,source,dll,document,label):
    modules=load_project(source,'_route_bounds_'+label)
    nav,models=modules['navmesh'],modules['models']
    native_type=nav.Native
    nav.Native=lambda unused_path:native_type(dll)
    reads={};calls=[];loads=[]
    class TracedMesh(nav.Mesh):
        def load(self,keys):
            entry={'context':dict(self.debug_context),'requested':sorted(keys)};loads.append(entry)
            try:return super().load(keys)
            finally:entry.update(loaded_tiles=len(self.loaded),loaded_bytes=self.bytes)
        def path(self,a,b,budget,*args,**kwargs):
            call={'start':xyz(a),'goal':xyz(b),'budget':budget,'context':dict(self.debug_context),'max_length_xy':kwargs.get('max_length_xy',args[0] if args else None)};calls.append(call)
            try:
                parts,count=super().path(a,b,budget,*args,**kwargs)
                route=models.Route(a,b,tuple(parts),count)
                call.update(status='success',expanded=count,length_xy=route.distance,point_count=len(route.points))
                return parts,count
            except models.PlanningError as error:
                call.update(status=error.code,expanded=getattr(error,'expanded',0),message=str(error));raise
    class Pack(nav.NavMeshPack):
        def read(self,entry):
            data=super().read(entry)
            if 'c'in entry:reads[entry['path']]={'path':entry['path'],'bytes':len(data),'sha256':sha(data)}
            return data
        @contextmanager
        def mesh(self,c,cancel=None):
            mesh=TracedMesh(self,c,cancel)
            try:yield mesh
            finally:mesh.close()
    manifest=Path(project)/'data/navigation/manifest.json'
    pack=Pack(manifest);pack.record_diagnostics=False
    pack.raw=dict(pack.raw,library={'path':str(Path(dll).resolve()),'bytes':Path(dll).stat().st_size,'sha256':sha(Path(dll).read_bytes())})
    point=lambda value:models.Point(**value)
    start,goal=point(document['start']),point(document['goal'])
    g=document.get('guide')
    guide=None if g is None else models.Route(start,goal,tuple(models.Segment(tuple(point(p)for p in s['points']),s['source'],s['reliability'],s['edge_id'])for s in g['segments']))
    began=time.monotonic();result={'label':label,'source_files':{name:file_info(Path(module.__file__))for name,module in modules.items()},'actual_dll':file_info(dll)}
    try:
        route=pack.plan(start,goal,guide=guide,blocked_edges=frozenset(document['blocked_edges']),max_expansions=document['max_expansions'])
        result.update(status='success',expanded=route.expanded,length_xy=route.distance,length_3d=sum(math.dist(xyz(a),xyz(b))for a,b in zip(route.points,route.points[1:])),points=[xyz(p)for p in route.points],all_segments_native=all(s.reliability=='navmesh'for s in route.segments),exact_endpoint_xy=(xyz(route.start)[:2]==xyz(start)[:2]and xyz(route.goal)[:2]==xyz(goal)[:2]))
    except models.PlanningError as error:
        result.update(status=error.code,expanded=getattr(error,'expanded',None),message=str(error),native_debug=parse_debug(str(error)))
    result.update(elapsed_seconds=time.monotonic()-began,calls=calls,loads=loads,verified_assets=list(reads.values()),diagnostic=pack.last_route_diagnostics)
    return result

def actual_case_checks(args, partial_report=None):
    project=Path(args.project_root).resolve(strict=True);manifest_path=project/'data/navigation/manifest.json'
    manifest_bytes=manifest_path.read_bytes();manifest=json.loads(manifest_bytes.decode('utf-8-sig'))
    plan_bytes=Path(args.plan).read_bytes();document=json.loads(plan_bytes.decode('utf-8-sig'))
    assets=read_json(args.regression_assets)
    require(assets.get('schema')==1,'Unexpected regression asset schema')
    require(sha(plan_bytes)==assets['reference_plan_sha256'],'Reference plan SHA differs from pinned regression')
    require(document['navigation_identity']==manifest['identity']==assets['navigation_identity'],'Navigation identity changed')
    require(sha(json.dumps(manifest['tiles'],sort_keys=True).encode())[:16]==manifest['identity'],'Manifest identity invalid')
    require(document.get('complete') is True and document.get('result',{}).get('status')=='error','Expected complete failed reference plan')
    indexed={entry['path']:entry for entry in manifest['maps']+manifest['tiles']}
    navroot=manifest_path.parent
    pinned=assets['maps']+assets['tiles']
    require(len(assets['tiles'])==46 and len(assets['maps'])==1,'Expected 46 required tiles and map0 header')
    for expected in pinned:
        require(indexed.get(expected['path'])==expected,'Manifest entry changed: '+expected['path'])
        path=(navroot/expected['path']).resolve()
        require(path.is_relative_to(navroot.resolve()),'Asset path escapes navigation root')
        data=path.read_bytes();require(len(data)==expected['bytes']and sha(data)==expected['sha256'],'Pinned asset mismatch: '+expected['path'])
    originals={manifest_path:sha(manifest_bytes),Path(args.plan):sha(plan_bytes)}
    for directory in (project/'adrive',Path(args.candidate_python)/'adrive' if (Path(args.candidate_python)/'adrive').is_dir() else Path(args.candidate_python)):
        for name in('__init__.py','navmesh.py','road_corridor.py','models.py','data.py'):
            path=directory/name;originals[path]=sha(path.read_bytes())
    for value in(args.dll,args.baseline_dll):originals[Path(value)]=sha(Path(value).read_bytes())
    print('ACTUAL_CASE: original planner and current DLL',flush=True)
    baseline=route_run(project,project,args.baseline_dll,document,'baseline')
    if partial_report is not None:partial_report['baseline']=baseline
    require(baseline['status']=='budget' and baseline.get('native_debug',{}).get('native_reason')=='node_pool_exhausted','Original recorded node-pool failure did not reproduce')
    print('ACTUAL_CASE: candidate planner and bounded-search DLL',flush=True)
    candidate=route_run(project,args.candidate_python,args.dll,document,'candidate')
    if partial_report is not None:partial_report['candidate']=candidate
    require(candidate['status']in('road_connection_unverified','success'),'Candidate did not finish with a verified route or a scoped road-connectivity result: '+candidate['status'])
    if candidate['status']=='success':
        require(candidate['all_segments_native']and candidate['exact_endpoint_xy']and candidate['points'],'Candidate exposed an incomplete or non-native route')
        require(candidate['diagnostic']['result']['status']=='success','Candidate route diagnostic disagrees')
    else:
        require(candidate['diagnostic']['result']['status']=='error'and 'segments'not in candidate['diagnostic']['result'],'Failure exposed a partial route')
    require(type(candidate['expanded'])is int and 0<=candidate['expanded']<=document['max_expansions'],'Candidate aggregate budget accounting is invalid')
    require(candidate['expanded']<baseline['expanded'],'Bounded search did not reduce expanded work for this regression')
    bounded=[c for c in candidate['calls']if c.get('max_length_xy')is not None]
    require(bounded,'Candidate never used bounded native calls')
    require(not any(parse_debug(c.get('message',''))and parse_debug(c['message']).get('native_reason')=='node_pool_exhausted'for c in candidate['calls']),'Candidate still exhausted a native node pool')
    verified={e['path']:e for record in(baseline,candidate)for e in record['verified_assets']}
    for path,entry in verified.items():
        expected=indexed.get(path)
        require(expected is not None and all(entry[k]==expected[k]for k in('path','bytes','sha256')),'Loaded asset not bound to original manifest')
        current=(navroot/path).read_bytes();require(len(current)==entry['bytes']and sha(current)==entry['sha256'],'Loaded asset changed before verification ended: '+path)
    for path,expected in originals.items():require(sha(path.read_bytes())==expected,'Input changed during verification: '+str(path))
    summary={'baseline_status':baseline['status'],'baseline_expanded':baseline['expanded'],'candidate_status':candidate['status'],'candidate_expanded':candidate['expanded'],'bounded_calls':len(bounded),'verified_asset_count':len(verified),'node_pool_exhausted_after':False,'full_manifest_retained':True,'all_inputs_unchanged':True,'swimming_authorized':False,'physical_bridge_repaired':False}
    return {'baseline':baseline,'candidate':candidate,'summary':summary,'verified_assets':list(verified.values())}

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for name in('dll','baseline-dll','fixture-dir','project-root','candidate-python','plan','regression-assets','report'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args(argv);report={'schema':1,'status':'running','scope':'synthetic additive ABI tests and actual recorded route with original manifest assets'}
    code=1
    try:
        require(__debug__,'Run validation without -O/PYTHONOPTIMIZE')
        report['native_checks']=native_checks(args)
        actual=actual_case_checks(args,report);report.update(actual)
        report['status']='passed';code=0
    except BaseException as error:
        report.update(status='failed',error=type(error).__name__+': '+str(error),traceback=traceback.format_exc())
    write_json(args.report,report)
    print(json.dumps({'status':report['status'],'report':str(args.report),'summary':report.get('summary'),'error':report.get('error')},ensure_ascii=False),flush=True)
    return code

if __name__=='__main__':raise SystemExit(main())
