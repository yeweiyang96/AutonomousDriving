"""Collision certificates exercised against real Detour, with separate floors."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import unittest

from adrive.models import Point,PlanningError
from adrive.navmesh import Mesh,NavMeshPack
from tests.test_navmesh import ROOT,entry

BUILD=ROOT/'runtime/motion-native-build/Release'


@unittest.skipUnless((BUILD/'adrive_navmesh.dll').exists(),'需要候选连续运动 DLL')
class MotionNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root=ROOT/'runtime/motion-fixtures';root.mkdir(parents=True,exist_ok=True)
        subprocess.run([str(BUILD/'navmesh_fixture.exe'),str(root)],check=True,capture_output=True)
        if not (root/'adrive_navmesh.dll').exists() or (root/'adrive_navmesh.dll').read_bytes()!=(BUILD/'adrive_navmesh.dll').read_bytes():
            shutil.copyfile(BUILD/'adrive_navmesh.dll',root/'adrive_navmesh.dll')
        raw=dict(schema=1,kind='adrive-ground-navmesh',mmap_version=16,identity='motion-fixture',
            client=dict(version='constructed-fixture'),settings=dict(walkable_climb_yards=.533333,agent_radius_yards=.533333),
            library=entry(root/'adrive_navmesh.dll',root),
            maps=[entry(root/f'{c:04}.mmap',root,c=c) for c in (0,1)],
            tiles=[entry(root/f'{c:04}_31_31.mmtile',root,c=c,row=31,col=31) for c in (0,1)])
        raw['tiles'].append(entry(root/'0001_31_30.mmtile',root,c=1,row=31,col=30))
        (root/'manifest.json').write_text(json.dumps(raw),encoding='utf-8')
        cls.pack=NavMeshPack(root/'manifest.json')

    def setUp(self):
        self.mesh=Mesh(self.pack,0);self.mesh.load({(31,31)});self.addCleanup(self.mesh.close)

    def test_whole_chord_rejects_wall_even_when_endpoints_are_valid(self):
        self.assertTrue(self.mesh.motion_validate((Point(0,15,5,0),),.1,()))
        self.assertTrue(self.mesh.motion_validate((Point(0,15,25,0),),.1,()))
        self.assertFalse(self.mesh.motion_validate((Point(0,15,5,0),Point(0,15,25,0)),.1,()))

    def test_floor_identity_prevents_xy_alias(self):
        low=(Point(0,2,2,0),Point(0,8,2,0));high=tuple(Point(p.c,p.x,p.y,5) for p in low)
        corridor=self.mesh.motion_corridor(low,2)
        self.assertTrue(self.mesh.motion_validate(low,.1,corridor))
        self.assertFalse(self.mesh.motion_validate(high,.1,corridor))
        self.assertFalse(self.mesh.motion_validate((low[0],high[1]),.1,()))

    def test_clearance_is_swept_and_water_remains_excluded(self):
        self.assertTrue(self.mesh.motion_validate((Point(0,15,1,0),Point(0,25,1,0)),.1,()))
        self.assertFalse(self.mesh.motion_validate((Point(0,15,.05,0),Point(0,25,.05,0)),.1,()))
        self.assertFalse(self.mesh.motion_validate((Point(0,5,15,0),),.1,()))

    def test_poly_portal_endpoint_does_not_change_floor(self):
        path=(Point(0,15,5,0),Point(0,20,5,0),Point(0,25,5,0))
        refs=self.mesh.motion_corridor(path,2)
        self.assertTrue(self.mesh.motion_validate(path,.1,refs))

    def test_cross_tile_requires_a_loaded_ground_connection(self):
        mesh=Mesh(self.pack,1);self.addCleanup(mesh.close);mesh.load({(31,31)})
        path=(Point(1,25,25,0),Point(1,25,35,0))
        self.assertFalse(mesh.motion_validate(path,.1,()))
        mesh.load({(31,30)})
        refs=mesh.motion_corridor(path,2)
        self.assertTrue(mesh.motion_validate(path,.1,refs))
