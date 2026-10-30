"""Finalize candidate metadata only; never activates or relaxes geometry."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

def data(value):return (json.dumps(value,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'\n').encode()
def finalize(root,validation_report=None):
    root=root.resolve();path=root/'manifest.json';raw=json.loads(path.read_bytes())
    rejected=[]
    if validation_report:
        report=json.loads(validation_report.read_bytes())
        if (not report['full_edge_validation'] or report['roads_manifest_sha256']!=hashlib.sha256(path.read_bytes()).hexdigest()
                or report['navigation_identity']!=raw['navigation_identity']
                or report['native_library_sha256']!=raw['native_library_sha256']):raise ValueError('Validation report does not bind this candidate')
        rejected=report['failures']
        if any('edge' not in row for row in rejected):raise ValueError('Cannot remove route/gate failures as road data')
        bad={row['edge'] for row in rejected};raw['edges']=[e for e in raw['edges'] if e['id'] not in bad]
        previous={r['edge']:r for r in raw.get('validation_exclusions',[])}
        previous.update({r['edge']:r for r in rejected});raw['validation_exclusions']=list(previous.values())
    discarded=[e['id'] for e in raw['edges'] if e['length']<=1e-5]
    raw['edges']=[e for e in raw['edges'] if e['length']>1e-5]
    active={e['id'] for e in raw['edges']}
    for entry in raw['shards'].values():
        p=(root/entry['path']).resolve()
        if not p.is_relative_to(root):raise ValueError('Shard escapes candidate')
        old=p.read_bytes()
        if hashlib.sha256(old).hexdigest()!=entry['sha256']:raise ValueError('Candidate shard hash mismatch')
        rows=[r for r in json.loads(old) if r['id'] in active];content=data(rows);p.write_bytes(content)
        entry.update(bytes=len(content),sha256=hashlib.sha256(content).hexdigest())
    if raw.get('validation'):
        for entry in raw['validation']['shards'].values():
            p=(root/entry['path']).resolve()
            if not p.is_relative_to(root):raise ValueError('Validation shard escapes candidate')
            old=p.read_bytes()
            if hashlib.sha256(old).hexdigest()!=entry['sha256']:raise ValueError('Validation shard hash mismatch')
            rows=[r for r in json.loads(old) if r['id'] in active];content=data(rows);p.write_bytes(content)
            entry.update(bytes=len(content),sha256=hashlib.sha256(content).hexdigest())
        raw['validation']['edge_count']=len(raw['edges'])
    floors=dict(schema=1,navigation_identity=raw['navigation_identity'],native_library_sha256=raw['native_library_sha256'],shards=raw['shards'])
    content=data(floors);(root/'floor-index.json').write_bytes(content)
    raw['floor_index']=dict(path='floor-index.json',sha256=hashlib.sha256(content).hexdigest())
    for source in raw['sources']:
        p=ROOT/source['path']
        if source['path'].startswith('addons/'):
            source['claimed_builds']=sorted(set(re.findall(r'1\.60\.1\.\d+',p.read_text(encoding='utf-8-sig'))))
            source['license_file']='addons/AzerothGPS/LICENSE.txt'
    raw['generator_sources']={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in
        ('tools/build_unified_roads.py','tools/build_topology_roads.py','tools/bind_unified_roads.py','tools/finalize_unified_roads.py',
         'adrive/road_sources.py','adrive/road_overlap.py','adrive/roads.py','native/navmesh/road_corridor_v1.inc')}
    raw.pop('identity',None);raw['identity']=hashlib.sha256(data(raw)).hexdigest()[:16];path.write_bytes(data(raw))
    audit=json.loads((root/'audit.json').read_bytes())
    audit['discarded_zero_length']=sorted(set(audit.get('discarded_zero_length',[]))|set(discarded))
    if raw.get('validation_exclusions'):audit['validation_exclusions']=raw['validation_exclusions']
    audit['published_edges']=len(raw['edges']);audit['road_identity']=raw['identity'];(root/'audit.json').write_bytes(data(audit))
    return raw['identity']

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('candidate',type=Path);p.add_argument('--validation-report',type=Path)
    args=p.parse_args();print(finalize(args.candidate,args.validation_report))
