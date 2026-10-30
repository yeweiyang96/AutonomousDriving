"""Run with WSL Python. Read local Forever DB2s; write only project output.

Uses the installed ForeverTracker MIT reader without changing its source/config.
Region labels are context, not altitude constraints or proof of a floor.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def token(name):
    value = 5381
    for byte in name.encode('utf-8'):
        value = (value*33+byte) & 0xFFFFFFFF
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dbc', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--tracker', type=Path, default=Path('/home/wangzekun/forever/integrations/ForeverTracker'))
    args = parser.parse_args()
    sys.path.insert(0, str(args.tracker/'tools'))
    from ft.db2 import Table
    nav = json.loads(args.manifest.read_text())
    provenance = json.loads((args.dbc.parent.parent/'provenance.json').read_text())
    if provenance['client']['version'] != nav['client']['version']:
        raise ValueError('Extracted client build differs from navigation build')
    sources, tables = [], {}
    for name in ('AreaTable', 'UiMap', 'UiMapAssignment'):
        path = args.dbc/(name+'.db2')
        table = Table(str(path))
        if table.layout_hash != int(table.meta['layoutHash'],16):
            raise ValueError('DB2 schema layout mismatch: '+name)
        skipped = sum(s[2] for s in table.sections if s[0] not in (0, 0x5452494E49545900))
        tables[name] = list(table)
        sources.append(dict(table=name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(), rows=len(tables[name]),
                            encrypted_records_skipped=skipped))
    labels = {}
    for row in tables['AreaTable']:
        if row['ContinentID'] in (0,1) and row['AreaName']:
            labels.setdefault(str(token(row['AreaName'])),set()).add(row['AreaName'])
    output = dict(schema=1, kind='forever-region-context', build=nav['client']['version'],
                  navigation_identity=nav['identity'], sources=sources,
                  reader_commit=subprocess.check_output(['git','-C',str(args.tracker),'rev-parse','HEAD'],text=True).strip(),
                  reader_license='MIT; ForeverTracker contributors',
                  labels={k:sorted(v) for k,v in labels.items()},
                  maps={str(r['ID']):r['Name'] for r in tables['UiMap']},
                  assignments=tables['UiMapAssignment'],
                  policy='Region names and broad assignment boxes are context only, never floor proof')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(build=output['build'], labels=len(labels), maps=len(output['maps']), sources=sources)))


if __name__ == '__main__':
    main()
