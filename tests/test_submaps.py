"""Submap alignment, alpha boundaries, zoom anchoring and pack rejection."""
import math
from pathlib import Path
import tempfile
import time
import tkinter as tk
import unittest
from PIL import Image
from adrive.models import Point
from adrive.mapview import Viewport, MapView
from adrive.data import DataBundle
from adrive.submaps import Submaps
from tools.export_submaps import layouts,warp_picture,write_pyramid

ROOT=Path(__file__).resolve().parents[1]


def manifest():
    return dict(schema=1,kind='wow-world-submap-overlays',tile_pixels=512,build={'version':'fixture'},
        area_count=1,tile_count=3,areas=[dict(id='cave-1',name='fixture',kind='cave',c=0,
        bounds=[0,0,64,64],north=64,west=64,pixels_per_yard=8,width=1024,height=512,
        tiles=[dict(n=1,col=0,row=0,path='a.webp'),dict(n=1,col=1,row=0,path='b.webp'),
               dict(n=2,col=0,row=0,path='c.webp')])])


class OverlayModelTests(unittest.TestCase):
    def test_same_transform_shared_edges_and_continent_culling(self):
        catalog=Submaps.from_manifest(ROOT,manifest())
        a=catalog.areas[0]['tiles'][1,0,0];b=catalog.areas[0]['tiles'][1,1,0]
        self.assertEqual(a.west-a.size,b.west)
        self.assertEqual(a.north,b.north)
        view=Viewport(0,32,32,8,800,600)
        self.assertTrue(catalog.visible(view))
        for tile,opacity,primary in catalog.visible(view):
            point=Point(tile.c,tile.north,tile.west)
            self.assertEqual(view.world(*view.screen(point)),point)
            self.assertGreater(opacity,0)
        view.c=1
        self.assertEqual(catalog.visible(view),[])

    def test_fade_is_continuous_and_deep_zoom_preserves_cursor_coordinate(self):
        catalog=Submaps.from_manifest(ROOT,manifest())
        view=Viewport(0,32,32,.04,800,600)
        area=catalog.areas[0]
        self.assertEqual(catalog.opacity(area,view),0)
        values=[]
        for i in range(100):
            view.scale=1+i*.05;values.append(catalog.opacity(area,view))
        self.assertEqual(values,sorted(values))
        self.assertLess(max(b-a for a,b in zip(values,values[1:])),.1)
        anchor=view.world(100,130)
        view.zoom(2,100,130,max_scale=32)
        self.assertAlmostEqual(view.world(100,130).x,anchor.x)
        self.assertAlmostEqual(view.world(100,130).y,anchor.y)
        self.assertGreater(catalog.zoom_limit(view,Point(0,32,32)),4)
        self.assertEqual(catalog.zoom_limit(view,Point(1,32,32)),4)

    def test_manifest_rejects_bad_bounds_paths_duplicates_and_infinite_scale(self):
        mutations=[lambda r:r['areas'][0].update(pixels_per_yard=math.inf),
            lambda r:r['areas'][0].update(bounds=[0,0,-1,10]),
            lambda r:r['areas'][0]['tiles'][0].update(path='../escape.webp'),
            lambda r:r['areas'][0]['tiles'][0].update(row=-1),
            lambda r:r['areas'][0]['tiles'].__setitem__(1,r['areas'][0]['tiles'][0]),
            lambda r:r.update(tile_count=99)]
        for mutate in mutations:
            value=manifest();mutate(value)
            with self.assertRaises(ValueError):Submaps.from_manifest(ROOT,value)


class OverlayExportTests(unittest.TestCase):
    def test_quarter_turn_and_outside_uv_are_transparent_not_repeated(self):
        source=Image.new('RGBA',(16,16),(255,0,0,255))
        # u=0..2 shows the source on the left half of the square only.
        patch,x,y=warp_picture(source,[1,0,0,16,0,2,0],8,8,1)
        self.assertEqual(patch.getpixel((3,8)),(255,0,0,255))
        self.assertEqual(patch.getpixel((13,8))[3],0)
        rotated,_,_=warp_picture(source,[1,0,0,16,math.pi/2,2,0],8,8,1)
        self.assertEqual(rotated.getpixel((8,3))[3],0)
        self.assertEqual(rotated.getpixel((8,13)),(255,0,0,255))

    def test_pyramid_uses_fixed_pixel_spacing_and_transparent_padding(self):
        parent=ROOT/'runtime/test-submaps';parent.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(dir=parent) as directory:
            root=Path(directory);image=Image.new('RGBA',(513,300),(20,30,40,255))
            area=dict(id='test',kind='city',name='test',c=0,bounds=[0,0,300,513])
            output=write_pyramid(image,root,root,area,300,513,1)
            tiles={(t['n'],t['col'],t['row']):t for t in output['tiles']}
            with Image.open(root/tiles[1,1,0]['path']) as tile:
                self.assertEqual(tile.convert('RGBA').getpixel((0,0))[3],255)
                self.assertEqual(tile.convert('RGBA').getpixel((1,0))[3],0)
            with Image.open(root/tiles[2,0,0]['path']) as tile:
                self.assertEqual(tile.convert('RGBA').getpixel((400,100))[3],0)

    def test_reference_primary_floors_and_two_underground_cities(self):
        result=layouts(ROOT/'addons/OpenMap')
        self.assertEqual({a['id'] for a in result if a['kind']=='city'},{'city-1455','city-1458'})
        self.assertNotIn('micro-34',{a['id'] for a in result})
        self.assertIn('micro-5',{a['id'] for a in result})
        self.assertTrue(all(len(p)==7 for a in result if 'pictures' in a for p in a['pictures']))


@unittest.skipUnless((ROOT/'data/submaps/manifest.json').exists(),'需要先导出子地图包')
class InstalledOverlayTests(unittest.TestCase):
    def test_ironforge_and_skull_rock_cover_real_world_coordinates(self):
        catalog=Submaps.load(ROOT/'data/submaps/manifest.json')
        for c,x,y,identity,scale in [(0,-4850,-1138,'city-1455',1.2),(1,1490,-4764,'micro-5',4)]:
            view=Viewport(c,x,y,scale,900,600)
            tiles=catalog.visible(view)
            self.assertTrue(any(t.area==identity and primary for t,opacity,primary in tiles))
        self.assertEqual(len(catalog.areas),281)

    def test_alpha_photo_variants_are_reused_when_panning_across_pixel_rounding(self):
        root=tk.Tk();root.withdraw()
        view=MapView(root,DataBundle.load(ROOT/'data'),lambda *_:None)
        view._initial_fit=False
        view.view=Viewport(0,-4850,-1138,.95,1100,720)
        try:
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                root.update();view.redraw()
                if not view._images.pending and not view._loading_tiles:break
                time.sleep(.01)
            self.assertEqual(view._shown_submaps,1)
            for i in range(12):
                view.view.west=-1138+i*.5/.95;view.redraw()
            photos={key:value[0] for key,value in view._overlay_photos.items()}
            for i in range(12):
                view.view.west=-1138+i*.5/.95;view.redraw()
            self.assertEqual(photos,{key:value[0] for key,value in view._overlay_photos.items()})
            self.assertLessEqual(view._overlay_photo_bytes,32*1024*1024)
            self.assertFalse(view._images.failed)
        finally:
            root.destroy()
