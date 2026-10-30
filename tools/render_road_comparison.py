"""Render real before/candidate road geometry over offline map data; no UI input."""
import argparse
import json
import math
from pathlib import Path
import sys
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from adrive.data import DataBundle
from adrive.maptiles import TilePyramid
from adrive.models import Point
from adrive.mapview import Viewport
from adrive.submaps import Submaps


def background(bundle,view):
    image=Image.new('RGBA',(view.width,view.height),'#13232d')
    pyramid=TilePyramid.from_bundle(bundle)
    tiles=[(t,1.) for t in pyramid.visible(view,pyramid.choose_level(view.c,view.scale))]
    submaps=Submaps.from_bundle(bundle)
    tiles.extend((t,opacity) for t,opacity,primary in submaps.visible(view) if primary)
    for tile,opacity in tiles:
        if not tile.path.exists():continue
        x,y=view.screen(Point(tile.c,tile.north,tile.west));size=tile.size*view.scale
        x0,y0,x1,y1=round(x),round(y),round(x+size),round(y+size)
        if x1<=x0 or y1<=y0:continue
        with Image.open(tile.path) as source:pixels=source.convert('RGBA').resize((x1-x0,y1-y0),Image.Resampling.BILINEAR)
        if opacity<1:pixels.putalpha(pixels.getchannel('A').point([round(a*opacity) for a in range(256)]))
        image.alpha_composite(pixels,(x0,y0));pixels.close()
    return image


def dashed(draw,points,color,width):
    on=True;remaining=5.
    for a,b in zip(points,points[1:]):
        length=math.dist(a,b)
        if not length:continue
        along=0.
        while along<length:
            step=min(remaining,length-along)
            if on:draw.line([(a[0]+(b[0]-a[0])*along/length,a[1]+(b[1]-a[1])*along/length),
                (a[0]+(b[0]-a[0])*(along+step)/length,a[1]+(b[1]-a[1])*(along+step)/length)],fill=color,width=width)
            along+=step;remaining-=step
            if remaining<1e-8:on=not on;remaining=5.


def paint(base,view,lines,title,fragmented=False):
    image=base.copy();draw=ImageDraw.Draw(image);visible=0;length=0.;coordinates=0
    for line in lines:
        if line['c']!=view.c:continue
        pts=line['points'];xy=[view.screen(Point(view.c,p[0],p[1])) for p in pts]
        xs=[p[0] for p in xy];ys=[p[1] for p in xy]
        if max(xs)<-20 or min(xs)>view.width+20 or max(ys)<-20 or min(ys)>view.height+20:continue
        visible+=1;coordinates+=len(pts)
        length+=sum(math.dist(a[:2],b[:2]) for a,b in zip(pts,pts[1:]))
        if line.get('kind','road')=='connector':dashed(draw,xy,'#a8b3b5',2);continue
        width=2 if view.scale<.12 else 3
        draw.line(xy,fill='#625f54',width=width+2,joint='curve')
        if fragmented:
            for x,y in (xy[0],xy[-1]):draw.ellipse((x-(width+2)/2,y-(width+2)/2,x+(width+2)/2,y+(width+2)/2),fill='#625f54')
        draw.line(xy,fill='#ded9c8',width=width,joint='curve')
        if fragmented:
            for x,y in (xy[0],xy[-1]):draw.ellipse((x-width/2,y-width/2,x+width/2,y+width/2),fill='#ded9c8')
    font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',16)
    draw.rectangle((12,10,view.width-12,42),fill='#14232d');draw.text((22,14),title,font=font,fill='#ded9c8')
    draw.rectangle((12,view.height-37,view.width-12,view.height-10),fill='#14232d')
    draw.text((22,view.height-34),f'离线真实数据绘制 · {visible} 条折线 / {coordinates} 个顶点',font=font,fill='#c2d1d9')
    return image,dict(visible_lines=visible,vertices=coordinates,full_polyline_length_xy=length)


def render(display,out,before):
    bundle=DataBundle.load(ROOT/'data');raw=json.loads(display.read_bytes());rows=raw['roads']
    old=json.loads((before/'manifest.json').read_bytes());oldlines=[]
    for entry in old['shards'].values():
        c=int(Path(entry['path']).stem.split('-')[0])
        for row in json.loads((before/entry['path']).read_bytes()):oldlines.append(dict(c=c,points=row['points']))
    out.mkdir(parents=True,exist_ok=True);report=[]
    scenes=[('continent',0,-5500.,-1000.,.04),('dun_morogh',0,-5500.,-2000.,.3),('ironforge',0,-4850.,-1138.,.95),
        ('dun_morogh_detail',0,-5050.,-1740.,.8),('ironforge_detail',0,-4990.,-1138.,1.8)]
    for name,c,north,west,scale in scenes:
        view=Viewport(c,north,west,scale,943,557);base=background(bundle,view)
        first,a=paint(base,view,oldlines,'旧：8码碎片路网',True);second,b=paint(base,view,rows,'新：连续道路与连接参考')
        first.save(out/f'{name}-before.png');second.save(out/f'{name}-after.png')
        combined=Image.new('RGB',(view.width*2+16,view.height),'#edf2f5');combined.paste(first,(0,0));combined.paste(second,(view.width+16,0))
        combined.save(out/f'{name}-comparison.png');report.append(dict(scene=name,view=vars(view),before=a,after=b))
    result=dict(scope='Static real geometry and offline map tiles; not a game walking test or a screenshot',
        candidate_display=str(display),old_road_identity=old['identity'],source_lines=len(rows),results=report)
    (out/'render-report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--display',type=Path,required=True)
    p.add_argument('--output',type=Path,default=ROOT/'runtime/road-display-performance/comparison')
    p.add_argument('--before',type=Path,default=ROOT/'data/roads/ef23594a993e2661')
    a=p.parse_args();render(a.display.resolve(),a.output.resolve(),a.before.resolve())
