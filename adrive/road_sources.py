"""Read literal reference road geometry; never execute reference addon Lua."""
from dataclasses import dataclass, field
import hashlib
import math
from pathlib import Path

from .lua_data import LiteralReader, array
from .models import Point


@dataclass
class SourceRoad:
    id: str
    points: list[Point]
    provenance: dict
    a: str | None = None
    b: str | None = None
    metadata: dict = field(default_factory=dict)


def unpack_points(edge, nodes):
    if not isinstance(edge[4], str):
        return list(zip(edge[4::2], edge[5::2]))
    alphabet = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz-_'
    values = {v:i for i,v in enumerate(alphabet)}
    x,y = nodes[(edge[0]-1)*2:(edge[0]-1)*2+2]
    result = [(x,y)]
    text = edge[4]
    if len(text)%4:
        raise ValueError('Truncated packed road')
    for i in range(0,len(text),4):
        a,b,c,d = (values[v] for v in text[i:i+4])
        x += a*64+b-2048; y += c*64+d-2048
        result.append((x,y))
    return result


def _height(value, offset=0., zero_unknown=True):
    # The overlay format uses zero for an unknown node floor. FloorTracks use
    # actual per-point heights and do not use this sentinel conversion.
    return float(value)+offset if isinstance(value,(int,float)) and not isinstance(value,bool) and (value!=0 or not zero_unknown) else None


def _metadata(graph_id, a, b, *, special='normal', joins=(), bridges=(),
              cave=False, indoor=False, capital=None, za=None, zb=None,
              offset=0., level=None, zero_unknown=True):
    restricted=cave or indoor
    join_a,join_b=a in joins,b in joins
    bridge_a,bridge_b=a in bridges,b in bridges
    return dict(graph_id=graph_id,original_a=a,original_b=b,special=special,
        join_a=join_a,join_b=join_b,bridge_a=bridge_a,bridge_b=bridge_b,
        bridge_allowed_a=not restricted or join_a or bridge_a,
        bridge_allowed_b=not restricted or join_b or bridge_b,
        cave=bool(cave),indoor=bool(indoor),capital=capital,
        source_height_a=za,source_height_b=zb,height_a=_height(za,offset,zero_unknown),height_b=_height(zb,offset,zero_unknown),
        height_offset=offset,source_level=level)


def _road_tables(name,reader):
    """Locate literal graph tables, retaining their world/level context."""
    if name=='Roads':
        obj=reader.table_after('ns','.','Roads','=')
        return [(c,obj[c],0.,c) for c in (0,1)]
    result=[];tokens=reader.tokens;continent=None;offset=0.;level=None;skip_until=-1
    def graphs(value):
        if not isinstance(value,dict):return
        if 'n' in value and 'e' in value:
            yield value
        else:
            for nested in value.values():yield from graphs(nested)
    for i,t in enumerate(tokens):
        if i<skip_until:continue
        if (t.value=='RoadOverlays' and i+3<len(tokens) and tokens[i+1].value=='['
                and tokens[i+2].value in (0,1)):
            continent=tokens[i+2].value;offset=0.;level=continent
        if t.value=='Roads' and i+3<len(tokens) and tokens[i+1].value=='[':
            level=tokens[i+2].value
        if t.value!='{' or not i or tokens[i-1].value!='=':continue
        try:
            reader.index=i;obj=reader._value()
        except ValueError:continue
        skip_until=reader.index
        if not isinstance(obj,dict):continue
        if 'base' in obj and 'zoff' in obj:
            continent=obj['base'];offset=obj['zoff']
        for graph in graphs(obj):
            if continent not in (0,1):raise ValueError('Unmapped overlay continent: '+name)
            result.append((continent,graph,offset,level))
    return result


def _floor_tracks(reader,roads,excluded):
    # Some older data snapshots have no FloorTracks assignment at all.
    anchor=('ns','.','FloorTracks','=')
    if not any(tuple(t.value for t in reader.tokens[i:i+len(anchor)])==anchor
               for i in range(len(reader.tokens)-len(anchor))):return
    tracks=reader.table_after(*anchor)
    for c in (0,1):
        for number,track in enumerate(array(tracks.get(c,{})),1):
            identity=f'agps:FloorTracks:{c}:{number}'
            if track.get('op','add')!='add':
                excluded.append(dict(id=identity,reason='not_added_floor_track'));continue
            pts=array(track.get('pts',{}));zs=array(track.get('zs',{}))
            if len(pts)<4 or len(pts)%2 or len(zs)!=len(pts)//2:
                raise ValueError('Malformed floor track: '+identity)
            if not all(isinstance(z,(int,float)) and not isinstance(z,bool) and math.isfinite(z) for z in zs):
                raise ValueError('Missing floor track height: '+identity)
            points=[Point(c,pts[i*2],pts[i*2+1],float(z)) for i,z in enumerate(zs)]
            graph_id=f'agps:FloorTracks:{c}:{number}'
            metadata=_metadata(graph_id,1,len(points),special='floor_track',joins=(1,len(points)),
                indoor=True,za=zs[0],zb=zs[-1],level=c)
            # Zero is a real FloorTrack height. Internal shape points are not
            # graph nodes and may never acquire arbitrary component bridges.
            metadata.update(height_a=points[0].z,height_b=points[-1].z,internal_bridge_allowed=False,
                point_heights=zs,track_time=track.get('time'),stair=bool(track.get('stair')))
            roads.append(SourceRoad(identity,points,dict(source='AzerothGPS',file='FloorTracks',group=number,
                original_source=6 if track.get('stair') else 2,height_hint=True),
                f'{graph_id}:1',f'{graph_id}:{len(points)}',metadata))


def source_roads(root, bundle):
    roads, sources, excluded = [], [], []
    path = root/'data/bundle.json'
    sources.append(dict(path='data/bundle.json',sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    for edge in bundle.edges:
        if edge.kind!='walk' or bundle.nodes[edge.a].c not in (0,1):
            excluded.append(dict(id=edge.id,reason='nonwalking',kind=edge.kind));continue
        roads.append(SourceRoad(edge.id,[bundle.nodes[edge.a],bundle.nodes[edge.b]],
            dict(source='existing',original_status=edge.status,original_source=edge.source),
            f'existing:{edge.a}',f'existing:{edge.b}',
            _metadata('existing',edge.a,edge.b,za=bundle.nodes[edge.a].z,zb=bundle.nodes[edge.b].z,
                level=bundle.nodes[edge.a].c,zero_unknown=False)))
    for name in ('Roads','Cities','Capitals','Caves','Indoors'):
        path = root/f'addons/AzerothGPS/Data/{name}.lua'
        reader = LiteralReader(path.read_text(encoding='utf-8-sig'))
        sources.append(dict(path=path.relative_to(root).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        tables=_road_tables(name,reader)
        for group,(c,obj,offset,level) in enumerate(tables):
            nodes=array(obj['n']);zs=obj.get('nz',obj.get('z',{}));drops=obj.get('drops',{})
            joins=set(array(obj.get('joins',{})));bridges=set(array(obj.get('bridge',{})))
            cave,indoor=bool(obj.get('cave')),bool(obj.get('indoor'))
            special=('indoor' if indoor else 'cave' if cave else 'city' if name=='Cities'
                     else 'capital' if obj.get('capital') else 'normal')
            graph_id=f'agps:{name}:{c}:{group}'
            for number,row in enumerate(array(obj['e']),1):
                e=array(row);identity=f'agps:{name}:{c}:{group}:{number}'
                if e[3] in (3,5,7) or number in drops:
                    excluded.append(dict(id=identity,reason='lift_swim_or_drop'));continue
                if name=='Roads' and e[3] not in (0,2,6):
                    excluded.append(dict(id=identity,reason='not_normal_road'));continue
                xy=unpack_points(e,nodes)
                if len(xy)<2:
                    excluded.append(dict(id=identity,reason='degenerate'));continue
                z1,z2=zs.get(e[0]),zs.get(e[1])
                h1,h2=_height(z1,offset),_height(z2,offset)
                known=h1 is not None and h2 is not None
                lengths=[0.]
                for a,b in zip(xy,xy[1:]):
                    lengths.append(lengths[-1]+math.dist(a,b))
                points=[Point(c,x,y,h1+(h2-h1)*lengths[i]/lengths[-1] if known and lengths[-1] else None)
                        for i,(x,y) in enumerate(xy)]
                roads.append(SourceRoad(identity,points,dict(source='AzerothGPS',file=name,group=group,
                    original_source=e[3],height_hint=known),f'{graph_id}:{e[0]}',f'{graph_id}:{e[1]}',
                    _metadata(graph_id,e[0],e[1],special=special,joins=joins,bridges=bridges,
                        cave=cave,indoor=indoor,capital=obj.get('capital'),za=z1,zb=z2,offset=offset,level=level)))
        if name=='Roads':_floor_tracks(reader,roads,excluded)
    return roads,sources,excluded
