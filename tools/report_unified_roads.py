"""Full source-to-release comparison with reproducible input coordinates."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from adrive.data import DataBundle
from adrive.road_sources import source_roads
from tools.build_unified_roads import encode,length_xy


def report(candidate,output):
    raw=json.loads((candidate/'manifest.json').read_bytes());audit=json.loads((candidate/'audit.json').read_bytes())
    sources,files,excluded=source_roads(ROOT,DataBundle.load(ROOT/'data'))
    if [(s['path'],s['sha256']) for s in files]!=[(s['path'],s['sha256']) for s in raw['sources']]:
        raise ValueError('Reference files changed since candidate generation')
    records={r.id:r for r in sources};published=defaultdict(list)
    for e in raw['edges']:
        for source in set(e['sources']):
            published[source].append(dict(edge=e['id'],selected_geometry=e['sources'][0]==source,
                entry=raw['nodes'][e['a']],exit=raw['nodes'][e['b']],length_xy=e['length'],
                validation_shard=raw['validation']['shards'][e['shard']]['path']))
    rows=[];counts=defaultdict(Counter)
    for row in audit['roads']:
        original=records[row['id']];group=original.provenance.get('file','existing')
        counts[group][row['status']]+=1
        rows.append({**row,'input_start':vars(original.points[0]),'input_goal':vars(original.points[-1]),
            'input_length_xy':length_xy(original.points),'published':published.get(original.id,[])})
    result=dict(road_identity=raw['identity'],navigation_identity=raw['navigation_identity'],
        native_library_sha256=raw['native_library_sha256'],source_files=raw['sources'],
        counts={k:dict(v) for k,v in counts.items()},source_count=len(rows),published_edges=len(raw['edges']),
        overlap_comparisons=dict(common_entry_exit_deduplicated=raw['counts'].get('deduplicated',raw['counts'].get('duplicate_whole_roads_removed',0)),
            geometry_updated_at_least_5_percent_shorter=raw['counts'].get('updated',0)),
        excluded_inputs=excluded,validation_exclusions=raw.get('validation_exclusions',[]),roads=rows,
        scope='Source processing status is separate from the collision validation of each published slice; no live movement proof')
    output.parent.mkdir(parents=True,exist_ok=True);output.write_bytes(encode(result))
    print(json.dumps({k:result[k] for k in ('road_identity','source_count','published_edges','counts','overlap_comparisons')},ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();report(a.candidate,a.output)
