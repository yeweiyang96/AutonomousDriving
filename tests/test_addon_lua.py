"""Execute production addon code in Lua 5.1 with a small public WoW API stub.

Optional development dependency: pip install -r requirements-dev.txt.
No game process, hardware, key events, network or screen access is used here.
"""
from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import unittest

from PIL import Image, ImageDraw

from adrive.models import Flags, Telemetry
from adrive.telemetry import decode_frame, decode_image, encode_frame

try:
    from lupa.lua51 import LuaRuntime
except ImportError:
    try:
        from lupa.luajit21 import LuaRuntime
    except ImportError:
        LuaRuntime = None


ROOT = Path(__file__).resolve().parents[1]
ADDON = ROOT / "addons" / "AutonomousDriving"
STUBS = r"""
state={
    x=-1234.5,y=5678.25,z=0,c=1,ui_map=1411,facing=math.pi/2,speed=7,
    connected=true,exists=true,combat=false,dead=false,swimming=false,flying=false,taxi=false,
    vehicle=false,position_available=true,fallback_available=true,fallback_x=-999,fallback_y=888,
    fallback_c=1,clock=123.456,unix_time=1790000000,
}
UIParent={}
SlashCmdList={}
function print(...) end
function GetTime() return state.clock end
function time() return state.unix_time end
function UnitPosition(unit)
    assert(unit=='player')
    if state.position_available then return state.x,state.y,state.z,state.c end
end
function GetPlayerFacing() return state.facing end
function GetUnitSpeed(unit) assert(unit=='player');return state.speed,state.speed end
function UnitIsConnected(unit) assert(unit=='player');return state.connected end
function UnitExists(unit) assert(unit=='player');return state.exists end
function UnitAffectingCombat(unit) assert(unit=='player');return state.combat end
function UnitIsDeadOrGhost(unit) assert(unit=='player');return state.dead end
function IsSwimming() return state.swimming end
function IsFlying() return state.flying end
function GetCurrentKeyBoardFocus() return state.focus end
function UnitOnTaxi(unit) assert(unit=='player');return state.taxi end
function UnitInVehicle(unit) assert(unit=='player');return state.vehicle end
C_Map={}
function C_Map.GetBestMapForUnit(unit) assert(unit=='player');return state.ui_map end
function C_Map.GetPlayerMapPosition(ui_map,unit)
    assert(ui_map==state.ui_map and unit=='player')
    if state.fallback_available then return {map_marker=true} end
end
function C_Map.GetWorldPosFromMapPos(ui_map,pos)
    assert(ui_map==state.ui_map and pos.map_marker)
    return state.fallback_c,{GetXY=function() return state.fallback_x,state.fallback_y end}
end
function CreateFrame(kind,name,parent)
    assert(kind=='Frame' and parent==UIParent)
    local f={scripts={},events={},textures={},shown=true}
    function f:SetFrameStrata(v) self.strata=v end
    function f:SetFrameLevel(v) self.level=v end
    function f:SetAlpha(v) self.alpha=v end
    function f:EnableMouse(v) self.mouse=v end
    function f:ClearAllPoints() self.point=nil end
    function f:SetPoint(...) self.point={...} end
    function f:SetSize(w,h) self.width=w;self.height=h end
    function f:Show() self.shown=true end
    function f:Hide() self.shown=false end
    function f:RegisterEvent(event) self.events[event]=true end
    function f:SetScript(event,fn) self.scripts[event]=fn end
    function f:CreateTexture(name,layer)
        local t={}
        function t:SetColorTexture(r,g,b,a) self.color={r,g,b,a} end
        function t:SetSize(w,h) self.width=w;self.height=h end
        function t:ClearAllPoints() self.point=nil end
        function t:SetPoint(...) self.point={...} end
        self.textures[#self.textures+1]=t
        return t
    end
    _G[name]=f
    return f
end
-- The reference addon must never call movement or hardware functions.
function MoveForwardStart() error('FORBIDDEN input') end
function MoveForwardStop() error('FORBIDDEN input') end
function TurnLeftStart() error('FORBIDDEN input') end
function TurnRightStart() error('FORBIDDEN input') end
AD={}
"""


class AddonHarness:
    def __init__(self, saved=None):
        self.lua = LuaRuntime(unpack_returned_tuples=True)
        self.lua.execute(STUBS)
        self.globals = self.lua.globals()
        self.state = self.globals.state
        if saved is not None:
            self.globals.AutonomousDrivingDB = self.lua.table_from(saved)
        load = self.lua.eval("function(source,name,namespace) return assert(loadstring(source))(name,namespace) end")
        load((ADDON / "Protocol.lua").read_text(encoding="utf-8"), "AutonomousDriving", self.globals.AD)
        load((ADDON / "AutonomousDriving.lua").read_text(encoding="utf-8"), "AutonomousDriving", self.globals.AD)
        self.frame = self.globals.AutonomousDrivingTelemetryFrame
        self.event("ADDON_LOADED", "AutonomousDriving")
        self.event("PLAYER_ENTERING_WORLD")

    def event(self, name, *args):
        self.frame.scripts["OnEvent"](self.frame, name, *args)

    def tick(self, elapsed=1 / 30):
        self.state.clock += elapsed
        self.frame.scripts["OnUpdate"](self.frame, elapsed)

    def packet(self):
        nibbles = []
        version = round(self.frame.textures[37].color[1]*15)*16 + round(self.frame.textures[38].color[1]*15)
        for index in range(33, 33 + (76 if version == 2 else 68)):
            color = self.frame.textures[index].color
            assert color[1] == color[2] == color[3] and color[4] == 1
            nibbles.append(round(color[1] * 15))
        return bytes(a * 16 + b for a, b in zip(nibbles[::2], nibbles[1::2]))

    def telemetry(self):
        return decode_frame(self.packet(), self.state.clock)

    def image(self):
        image = Image.new("RGB", (int(self.frame.width), int(self.frame.height)))
        draw = ImageDraw.Draw(image)
        for index in range(1, 129):
            texture = self.frame.textures[index]
            x, y = int(texture.point[4]), -int(texture.point[5])
            color = tuple(math.floor(texture.color[channel] * 255 + 0.5) for channel in (1, 2, 3))
            draw.rectangle((x, y, x + int(texture.width) - 1, y + int(texture.height) - 1), fill=color)
        return image

    def encode(self, telemetry):
        data = {name: getattr(telemetry, name) for name in (
            "session", "seq", "c", "ui_map", "x", "y", "z", "facing", "speed", "flags")}
        data["flags"] = int(data["flags"])
        result = self.globals.AD.Protocol.Encode(self.lua.table_from(data))
        if isinstance(result, tuple):
            raise ValueError(result[1])
        return bytes(result[i] for i in range(1, len(result) + 1))


@unittest.skipIf(LuaRuntime is None, "可选 Lua 执行验证需要 requirements-dev.txt 中的 Lupa（Lua 5.1/LuaJIT）")
class AddonLuaTests(unittest.TestCase):
    def setUp(self):
        self.addon = AddonHarness()

    def test_actual_lua51_load_and_rendered_frame(self):
        self.assertEqual("Lua 5.1", self.addon.lua.eval("_VERSION"))
        t = self.addon.telemetry()
        self.assertEqual((1, 1411, -1234.5, 5678.25), (t.c, t.ui_map, t.x, t.y))
        self.assertEqual(Flags.VALID, t.flags)
        self.assertEqual((128, 64), (self.addon.frame.width, self.addon.frame.height))
        self.assertFalse(self.addon.frame.mouse)
        self.assertEqual(128, len(self.addon.frame.textures))
        self.assertEqual(t, decode_image(self.addon.image(), t.captured_at))

    def test_actual_lua_encoder_matches_python_golden_and_rounding(self):
        golden = Telemetry(0x12345678, 0xFFFFFFFE, 1, 1411, -1234.5, 5678.25, math.pi / 2, 7, Flags.VALID)
        self.assertEqual(bytes.fromhex("4144010078563412feffffff010083050000d8b2ffffe4620100004000070100066c"),
                         self.addon.encode(golden))
        for x in (-17000.123, -0.03125, 0.03125, 17000.987):
            for angle in (-0.001, math.tau - 0.000001, math.tau + 0.5):
                t = replace(golden, x=x, y=-x, facing=angle, flags=Flags.VALID | Flags.COMBAT)
                self.assertEqual(encode_frame(t), self.addon.encode(t))

    def test_lua_encoder_rejects_invalid_values(self):
        t = Telemetry(1, 1, 0, 1, 0, 0, 0, 7, Flags.VALID)
        for changes in ({"speed": -1}, {"x": math.inf}, {"facing": math.nan}, {"flags": Flags(1024)}):
            with self.assertRaises(ValueError):
                self.addon.encode(replace(t, **changes))

    def test_sampling_rate_and_loading_continuously_invalidate(self):
        before = self.addon.telemetry().seq
        self.addon.tick(0.01)
        self.addon.tick(0.01)
        self.assertEqual(before, self.addon.telemetry().seq)
        self.addon.tick(0.014)
        self.assertEqual(before + 1, self.addon.telemetry().seq)
        self.addon.event("PLAYER_LEAVING_WORLD")
        loading = self.addon.telemetry()
        self.assertEqual(Flags.LOADING, loading.flags)
        self.assertEqual((0, 0, 0, 0), (loading.x, loading.y, loading.facing, loading.speed))
        self.addon.tick()
        self.assertGreater(self.addon.telemetry().seq, loading.seq)
        self.assertFalse(self.addon.telemetry().flags & Flags.VALID)
        self.addon.event("PLAYER_ENTERING_WORLD")
        self.assertTrue(self.addon.telemetry().flags & Flags.VALID)

    def test_combat_death_swimming_flying_taxi_and_vehicle_bits(self):
        mapping = {"combat": Flags.COMBAT, "dead": Flags.DEAD, "swimming": Flags.SWIMMING,
                   "flying": Flags.FLYING, "taxi": Flags.TAXI, "vehicle": Flags.TRANSPORT}
        for key, flag in mapping.items():
            with self.subTest(key=key):
                self.addon.state[key] = True
                self.addon.tick()
                self.assertEqual(Flags.VALID | flag, self.addon.telemetry().flags)
                self.addon.state[key] = False

    def test_focus_disconnect_and_missing_required_api(self):
        self.addon.state.focus = self.addon.lua.table_from({"name": "ChatFrameEditBox"})
        self.addon.tick()
        self.assertEqual(Flags.VALID | Flags.INPUT_FOCUS, self.addon.telemetry().flags)
        self.addon.state.focus = None
        self.addon.state.connected = False
        self.addon.tick()
        self.assertEqual(Flags.LOADING, self.addon.telemetry().flags)
        self.addon.state.connected = True
        self.addon.globals.IsFlying = None
        self.addon.tick()
        self.assertFalse(self.addon.telemetry().flags & Flags.VALID)
        self.assertFalse(self.addon.globals.AD.statusKnown)
        self.assertEqual((0, 0), (self.addon.telemetry().x, self.addon.telemetry().y))

    def test_missing_focus_api_also_blocks_validity(self):
        self.addon.globals.GetCurrentKeyBoardFocus = None
        self.addon.tick()
        self.assertFalse(self.addon.telemetry().flags & Flags.VALID)

    def test_failed_focus_api_is_not_treated_as_no_focus(self):
        self.addon.lua.execute("GetCurrentKeyBoardFocus=function() error('restricted focus') end")
        self.addon.tick()
        telemetry = self.addon.telemetry()
        self.assertFalse(telemetry.flags & Flags.VALID)
        self.assertEqual((0, 0), (telemetry.x, telemetry.y))
        self.assertFalse(self.addon.globals.AD.statusKnown)

    def test_world_coordinate_fallback_not_normalized_map_position(self):
        self.addon.state.position_available = False
        self.addon.tick()
        t = self.addon.telemetry()
        self.assertEqual((1, -999, 888), (t.c, t.x, t.y))
        self.assertTrue(t.flags & Flags.VALID)
        self.addon.state.fallback_available = False
        self.addon.tick()
        self.assertFalse(self.addon.telemetry().flags & Flags.VALID)
        self.assertEqual((0, 0), (self.addon.telemetry().x, self.addon.telemetry().y))

    def test_nil_facing_and_failed_essential_api_never_retain_old_frame(self):
        self.addon.state.facing = None
        self.addon.tick()
        self.assertFalse(self.addon.telemetry().flags & Flags.VALID)
        self.assertEqual(0, self.addon.telemetry().facing)
        self.addon.state.facing = 0
        self.addon.lua.execute("UnitAffectingCombat=function() error('restricted') end")
        self.addon.tick()
        self.assertFalse(self.addon.telemetry().flags & Flags.VALID)

    def test_reload_changes_session_and_layout_commands_work(self):
        previous = self.addon.telemetry().session
        saved = dict(self.addon.globals.AutonomousDrivingDB)
        reloaded = AddonHarness(saved)
        self.assertNotEqual(previous, reloaded.telemetry().session)
        reloaded.globals.SlashCmdList.AUTONOMOUSDRIVINGTELEMETRY("cell 12")
        self.assertEqual((192, 96), (reloaded.frame.width, reloaded.frame.height))
        reloaded.globals.SlashCmdList.AUTONOMOUSDRIVINGTELEMETRY("hide")
        self.assertFalse(reloaded.frame.shown)
        reloaded.globals.SlashCmdList.AUTONOMOUSDRIVINGTELEMETRY("reset")
        self.assertTrue(reloaded.frame.shown)
        self.assertEqual((128, 64), (reloaded.frame.width, reloaded.frame.height))


if __name__ == "__main__":
    unittest.main()
