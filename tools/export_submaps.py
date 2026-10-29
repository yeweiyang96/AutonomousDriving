"""Export transparent city/cave overlays using OpenMap's world-space placement.

Art is read from the installed client. Layout provenance remains explicitly the
reference addon's version; reading current art does not re-date its coordinates.
"""
from __future__ import annotations
import argparse
from collections import Counter
from datetime import datetime, timezone
import io
import json
import math
from pathlib import Path
import sys
from PIL import Image, ImageChops

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
from adrive.lua_data import LiteralReader,array
from tools.casc_storage import CascStorage
from tools.export_maps import installed_build,digest,atomic_json


def layouts(reference):
    mini=LiteralReader((reference/'Data/CaveMinimap.lua').read_text(encoding='utf-8-sig'))
    result=[]
    for kind,table in [('cave',mini.table_after('ns','.','MapCaveMinimap','=')),
                       ('city',mini.table_after('ns','.','MapCityMinimap','='))]:
        for key,value in sorted(table.items()):
            c,x0,y0,x1,y1,pictures=array(value)
            pictures=array(pictures)
            if len(pictures)%7: raise ValueError('Invalid WMO picture stride')
            result.append(dict(id=f'{kind}-{key}',kind=kind,c=c,bounds=[x0,y0,x1,y1],
                name={1455:'铁炉堡',1458:'幽暗城'}.get(key,f'洞穴 {key}'),
                pictures=[pictures[i:i+7] for i in range(0,len(pictures),7)]))
    micro=LiteralReader((reference/'Data/Caves.lua').read_text(encoding='utf-8-sig')).table_after('ns','.','MapCaves','=')
    seen=set()
    for key,value in sorted(micro.items()):
        name,c,x0,y0,x1,y1,u0,v0,u1,v1,tiles=array(value)
        # Match OpenMap's primary floor policy; deeper Blackrock floors are not
        # overlaid simultaneously over its main map.
        if key in (34,35) or name in seen: continue
        seen.add(name)
        if [u0,v0,u1,v1] != [0,0,1,1]: raise ValueError('Unsupported micro-map crop')
        result.append(dict(id=f'micro-{key}',kind='micro',c=c,name=name,bounds=[x0,y0,x1,y1],
                           files=array(tiles),mask=f'Media/Textures/MapCave/{key}.tga'))
    return result


def warp_picture(source, picture, north, west, scale):
    """WoW positive rotation is counter-clockwise, in screen coordinates.

    UV outside 0..1 remains transparent (CLAMPTOBLACKADDITIVE), never repeated
    or stretched. Output is a small rotated patch, not the entire region.
    """
    fid,x,y,side,angle,u1,v0=picture
    if not side>0 or u1<1 or v0>0: raise ValueError('Unsupported WMO UV extent')
    cs,sn=math.cos(angle),math.sin(angle)
    cx,cy=(west-y)*scale,(north-x)*scale
    size=side*scale
    half=size*(abs(cs)+abs(sn))/2
    left,top=math.floor(cx-half),math.floor(cy-half)
    right,bottom=math.ceil(cx+half),math.ceil(cy+half)
    sx,sy=source.width*u1/size,source.height*(1-v0)/size
    coeff=(cs*sx,-sn*sx,source.width*u1/2+sx*(cs*(left-cx)-sn*(top-cy)),
           sn*sy,cs*sy,source.height*(v0+(1-v0)/2)+sy*(sn*(left-cx)+cs*(top-cy)))
    patch=source.transform((right-left,bottom-top),Image.Transform.AFFINE,coeff,
                           Image.Resampling.BICUBIC,fillcolor=(0,0,0,0))
    return patch,left,top


def compose_wmo(area,read):
    scale=2.0  # 256px / 128 yards for the city's native WMO tiles.
    # Preserve smaller, denser tunnel pictures, up to the desktop's deep zoom.
    scale=min(4.,max(min(read(p[0]).width*p[5]/p[3],read(p[0]).height*(1-p[6])/p[3]) for p in area['pictures']))
    corners=[]
    for _,x,y,side,angle,_,_ in area['pictures']:
        half=side*(abs(math.cos(angle))+abs(math.sin(angle)))/2
        corners.extend(((x-half,y-half),(x+half,y+half)))
    north=math.ceil(max(x for x,y in corners)*scale)/scale
    west=math.ceil(max(y for x,y in corners)*scale)/scale
    width=math.ceil((west-min(y for x,y in corners))*scale)
    height=math.ceil((north-min(x for x,y in corners))*scale)
    if width*height>64*1024*1024: raise ValueError('Overlay exceeds composition budget')
    out=Image.new('RGBA',(width,height))
    for picture in area['pictures']:
        patch,left,top=warp_picture(read(picture[0]),picture,north,west,scale)
        out.alpha_composite(patch,(left,top));patch.close()
    # Crop transparent margins while preserving the same world transform.
    box=out.getbbox()
    if box is None: raise ValueError('Empty WMO overlay '+area['id'])
    cropped=out.crop(box);out.close()
    return cropped,north-box[1]/scale,west-box[0]/scale,scale


def compose_micro(area,read,reference):
    if len(area['files'])!=12: raise ValueError('Expected twelve micro-map tiles')
    canvas=Image.new('RGBA',(1024,768))
    for i,fid in enumerate(area['files']):
        source=read(fid)
        if source.size!=(256,256): raise ValueError('Invalid micro-map tile dimensions')
        canvas.paste(source,((i%4)*256,(i//4)*256))
    image=canvas.crop((0,0,1002,668));canvas.close()
    with Image.open(reference/area['mask']) as raw:
        mask=raw.convert('RGBA').getchannel('A').resize(image.size,Image.Resampling.LANCZOS)
    image.putalpha(ImageChops.multiply(image.getchannel('A'),mask));mask.close()
    x0,y0,x1,y1=area['bounds']
    scale=min(4.,1002/(y1-y0),668/(x1-x0))
    out=image.resize((math.ceil((y1-y0)*scale),math.ceil((x1-x0)*scale)),Image.Resampling.LANCZOS)
    image.close()
    return out,x1,y1,scale


def write_pyramid(image,folder,root,area,north,west,scale):
    result=[]
    n=1
    level=Image.new('RGBA',(math.ceil(image.width/512)*512,math.ceil(image.height/512)*512))
    level.paste(image,(0,0))
    while True:
        for row in range(math.ceil(level.height/512)):
            for col in range(math.ceil(level.width/512)):
                tile=level.crop((col*512,row*512,col*512+512,row*512+512))
                if tile.getbbox():
                    path=folder/area['id']/str(n)/f'{col}_{row}.webp'
                    path.parent.mkdir(parents=True,exist_ok=True)
                    tile.save(path,'WEBP',lossless=True,exact=True,method=3)
                    data=path.read_bytes()
                    result.append(dict(n=n,col=col,row=row,path=path.relative_to(root).as_posix(),
                                       bytes=len(data),sha256=digest(data)))
                tile.close()
        if max(level.size)<=512: break
        size=(math.ceil(level.width/2),math.ceil(level.height/2))
        reduced=level.resize(size,Image.Resampling.LANCZOS)
        level.close()
        level=reduced;n*=2
    level.close()
    return dict(id=area['id'],kind=area['kind'],name=area['name'],c=area['c'],
                bounds=area['bounds'],north=north,west=west,pixels_per_yard=scale,
                width=image.width,height=image.height,tiles=result)


def export(game,reference,library,output):
    build=installed_build(game,'wowf')
    generation=build['version']+'-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    folder=output/generation;folder.mkdir(parents=True)
    sources=folder/'sources';sources.mkdir()
    definitions=layouts(reference)
    source_files=['Data/CaveMinimap.lua','Data/Caves.lua','Map/Close.lua','LICENSE.txt','Media/Textures/NOTICE.md',
                  *sorted({a['mask'] for a in definitions if 'mask' in a})]
    inputs=[]
    for name in source_files:
        content=(reference/name).read_bytes()
        dest=sources/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(content)
        inputs.append(dict(path=name,sha256=digest(content),bytes=len(content)))
    textures={};cache={};areas=[]
    with CascStorage(game,library,'wowf') as storage:
        def read(fid):
            if fid not in cache:
                data=storage.read(fid)
                with Image.open(io.BytesIO(data)) as source:
                    source.load();image=source.convert('RGBA')
                if max(image.size)>1024:raise ValueError('Oversized source image')
                cache[fid]=image
                textures[fid]=dict(file_id=fid,sha256=digest(data),bytes=len(data),size=list(image.size))
            return cache[fid]
        for i,area in enumerate(definitions,1):
            image,north,west,scale=(compose_micro(area,read,reference) if area['kind']=='micro' else compose_wmo(area,read))
            areas.append(write_pyramid(image,folder,output,area,north,west,scale));image.close()
            # Keep decoded source memory bounded even across many caves.
            for im in cache.values():im.close()
            cache.clear()
            if i%25==0 or area['kind']=='city':print(f'{i}/{len(definitions)} {area["id"]}',flush=True)
    if installed_build(game,'wowf')!=build:raise RuntimeError('Game version changed during export')
    raw=dict(schema=1,kind='wow-world-submap-overlays',tile_pixels=512,build=build,
             source_directory=sources.relative_to(output).as_posix(),
             layout_source=dict(addon='OpenMap v0.7.5-beta',wmo_layout_build='1.60.1.70245',
                 note='Reference placement and primary-floor policy reused; art exported from current client. Coordinates were not regenerated from current WMO geometry.',files=inputs),
             textures=list(textures.values()),areas=areas,area_count=len(areas),
             tile_count=sum(len(a['tiles']) for a in areas),bytes=sum(t['bytes'] for a in areas for t in a['tiles']),
             counts=dict(Counter(a['kind'] for a in areas)))
    atomic_json(folder/'root-relative-manifest.json',raw)
    from tools.check_submaps import check
    report=check(output,raw);atomic_json(folder/'validation.json',report)
    atomic_json(output/'manifest.json',raw)
    (output/'NOTICE.txt').write_text('Original World of Warcraft artwork belongs to Blizzard Entertainment.\n'
        'OpenMap placement data, masks and source notices are retained under the versioned sources directory.\n'
        'Current client textures and older reference placement are recorded separately in manifest.json.\n',encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--game-root',type=Path,default=Path(r'C:\Program Files (x86)\World of Warcraft'))
    p.add_argument('--reference',type=Path,default=ROOT/'addons/OpenMap')
    p.add_argument('--library',type=Path,default=ROOT/'runtime/map-tools/build/Release/CascLib.dll')
    p.add_argument('--output',type=Path,default=ROOT/'data/submaps')
    a=p.parse_args();export(a.game_root,a.reference,a.library,a.output)
