"""Read-only motion/USB timeline diagnostics. Never opens a device or socket."""
import argparse
from collections import Counter
import json
from pathlib import Path


def summarize(path):
    windows=[];statuses=[];decisions=[];issues=[];events=set();counts=Counter()
    for number,line in enumerate(path.read_text(encoding='utf-8').splitlines(),1):
        if not line.strip():continue
        row=json.loads(line);kind=row.get('kind');counts[kind]+=1
        if kind=='motion_window':
            window=row['window'];source=window['source'];frames=window['frames'];duration=sum(f['duration_ms'] for f in frames)
            entry=dict(line=number,time=row.get('t'),revision=row['firmware_revision'],route=window['route_version'],
                model=window['model_version'],source_frame=source['seq'],source_session=source['session'],speed=source['speed'],
                source_age_ms=row['source_age_ms'],calculation_ms=row['calculation_ms'],duration_ms=duration,frames=frames)
            entry['firmware_identity']=row.get('firmware_identity')
            windows.append(entry)
            if duration>200 or any(f['duration_ms']<20 for f in frames):issues.append(dict(line=number,reason='invalid_duration'))
            if row['source_age_ms']>80:issues.append(dict(line=number,reason='stale_authorization'))
            if any(not f['keys'] for f in frames):issues.append(dict(line=number,reason='all_released_in_motion'))
            event=window.get('event_id',0);key=window['route_version'],event
            if event:
                if key in events:issues.append(dict(line=number,reason='repeated_one_shot'))
                events.add(key)
        elif kind=='pico_lease':
            status=row['status']
            if 'revision' in status:statuses.append(dict(time=row.get('t'),**status))
        elif kind in ('motion_decision','replan'):
            decisions.append(dict(line=number,**row))
    # Configuration is present in the session record; use those bindings rather
    # than assuming that every user left A/D/W/Q/E unchanged.
    settings={}
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.strip():
            row=json.loads(line)
            if row.get('kind')=='session':settings=row.get('settings',{});break
    codes={chr(65+i):4+i for i in range(26)};codes.update({'[':47,']':48,'/':56})
    codes.update(UP=82,DOWN=81,LEFT=80,RIGHT=79,NUMPAD_DIVIDE=84)
    moving={codes.get(settings.get(n,d).upper()) for n,d in (('forward_key','W'),('strafe_left_key','Q'),('strafe_right_key','E'))}
    turning={codes.get(settings.get(n,d).upper()) for n,d in (('left_key','A'),('right_key','D'))}
    for window in windows:
        if any(set(f['keys'])&turning and not set(f['keys'])&moving for f in window['frames']):
            issues.append(dict(line=window['line'],reason='pivot_without_translation'))
    # A replaced short window may legitimately have no response before the next
    # one. Report evidence absence, not proof of a transport/game failure.
    observed_revisions={(s['boot'],s['epoch'],s['session'],s['submitted_revision']) for s in statuses}
    missing=[]
    for w in windows:
        identity=w.get('firmware_identity') or {}
        key=tuple(identity.get(n) for n in ('boot','epoch','session'))+(w['revision'],)
        if key not in observed_revisions:missing.append(dict(identity=identity,revision=w['revision']))
    return dict(log=str(path.resolve()),counts=dict(counts),windows=windows,statuses=statuses,decisions=decisions,issues=issues,
        revisions_without_observed_usb_submission=missing,
        evidence='Recorded USB status only; absence is not failure, submission/completion is not game movement')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('log',type=Path);parser.add_argument('--output',type=Path)
    args=parser.parse_args();result=summarize(args.log)
    if args.output:args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('windows','statuses')},ensure_ascii=False,indent=2))
