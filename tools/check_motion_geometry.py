"""Native motion ABI regression on existing bridge/cross-tile geometry."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from adrive.models import Point
from adrive.motion_worker import SurfaceValidator
from tools.check_navigation_regressions import KNOWN_CASES


def check(manifest,report):
    validator=SurfaceValidator(manifest);rows=[]
    try:
        for case in KNOWN_CASES[:6]:
            a,b=Point(case['continent'],*case['start']),Point(case['continent'],*case['goal'])
            route=validator.pack.plan(a,b)
            validator.prepare(route.points)
            # Certify a point on each original segment. These are source-floor
            # checks, not a claim that a shortest funnel has tracking clearance.
            points=route.points
            certified=sum(validator((p,),.1) for p in points)
            if not validator((points[0],),.1) or not validator((points[-1],),.1):
                raise ValueError(f"{case['name']}: endpoint lost floor/clearance")
            rows.append(dict(name=case['name'],points=len(points),source_points_with_tracking_clearance=certified,
                polygons=len(validator.corridor),status='passed'))
    finally:validator.close()
    result=dict(status='passed',native_sha256=validator.pack.raw['library']['sha256'],
        navigation_identity=validator.pack.identity,cases=rows,scope='static floor/corridor ABI only; no game input')
    report.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--manifest',type=Path,required=True);p.add_argument('--report',type=Path,required=True)
    a=p.parse_args();check(a.manifest,a.report)
