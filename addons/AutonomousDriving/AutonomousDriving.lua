-- This addon only displays state. All physical input is performed by the
-- explicitly started desktop application through the user's Pico firmware.
local addonName, AD = ...
local P=AD.Protocol
local F={VALID=1,COMBAT=2,DEAD=4,SWIMMING=8,FLYING=16,LOADING=32,INPUT_FOCUS=64,TAXI=128,TRANSPORT=256}
local frame=CreateFrame('Frame','AutonomousDrivingTelemetryFrame',UIParent)
frame:SetFrameStrata('TOOLTIP')
frame:SetFrameLevel(10000)
frame:SetAlpha(1)
frame:EnableMouse(false)
local cells={}
for i=1,P.COLS*P.ROWS do
    local texture=frame:CreateTexture(nil,'OVERLAY')
    texture:SetColorTexture(0,0,0,1)
    cells[i]=texture
end
local initialized,loading,session,seq,elapsed=false,true,0,0,0
local function accessible(value)
    if type(issecretvalue)=='function' and issecretvalue(value) then return false end
    if type(canaccessvalue)=='function' and not canaccessvalue(value) then return false end
    return true
end
local function number(value)
    return accessible(value) and type(value)=='number' and value==value and value~=math.huge and value~=-math.huge
end
local function call(fn,...)
    if type(fn)~='function' then return nil end
    local ok,a,b,c,d=pcall(fn,...)
    if ok then return a,b,c,d end
end
local function yes(fn,...)
    local value=call(fn,...)
    return accessible(value) and value==true
end
local function boolean(value)
    return accessible(value) and type(value)=='boolean'
end
local function color(index,value)
    cells[index]:SetColorTexture(value[1]/255,value[2]/255,value[3]/255,1)
end
local function layout()
    local db=AutonomousDrivingDB
    local size=math.max(4,math.min(24,tonumber(db.cell) or 8))
    db.cell=size
    frame:ClearAllPoints()
    frame:SetPoint('TOPLEFT',UIParent,'TOPLEFT',tonumber(db.x) or 8,-(tonumber(db.y) or 8))
    frame:SetSize(P.COLS*size,P.ROWS*size)
    for i,texture in ipairs(cells) do
        texture:SetSize(size,size)
        texture:ClearAllPoints()
        texture:SetPoint('TOPLEFT',frame,'TOPLEFT',((i-1)%P.COLS)*size,-math.floor((i-1)/P.COLS)*size)
    end
    for i=1,16 do
        color(i,P.Markers[i])
        color(16+i,{(i-1)*17,(i-1)*17,(i-1)*17})
        color(112+i,P.Markers[17-i])
    end
    frame:Show()
end
local function mapPosition(uiMap)
    if not (number(uiMap) and C_Map and C_Map.GetPlayerMapPosition and C_Map.GetWorldPosFromMapPos) then return nil end
    local pos=call(C_Map.GetPlayerMapPosition,uiMap,'player')
    if not accessible(pos) or not pos then return nil end
    local continent,world=call(C_Map.GetWorldPosFromMapPos,uiMap,pos)
    if not (accessible(world) and world and number(continent)) then return nil end
    local ok,north,west=pcall(function() return world:GetXY() end)
    if ok and number(north) and number(west) then return north,west,continent end
end
local function sample()
    local flags=0
    local uiMap=call(C_Map and C_Map.GetBestMapForUnit,'player')
    if not number(uiMap) or uiMap<0 then uiMap=0 end
    local x,y,z,c=call(UnitPosition,'player')
    if not (number(x) and number(y) and number(c)) then x,y,c=mapPosition(uiMap); z=nil end
    if not number(z) or math.abs(z)>=134217727 then z=nil end
    local facing=call(GetPlayerFacing)
    local speed=call(GetUnitSpeed,'player')
    local connected=call(UnitIsConnected,'player')
    local exists=call(UnitExists,'player')
    local combat=call(UnitAffectingCombat,'player')
    local dead=call(UnitIsDeadOrGhost,'player')
    local swimming=call(IsSwimming)
    local flying=call(IsFlying)
    local taxi=call(UnitOnTaxi,'player')
    -- A successful nil result means no edit box has focus. A failed call must
    -- not be confused with nil, otherwise keyboard input could enter chat.
    local focusKnown,focus=false,nil
    if type(GetCurrentKeyBoardFocus)=='function' then focusKnown,focus=pcall(GetCurrentKeyBoardFocus) end
    local statusKnown=boolean(connected) and boolean(exists) and boolean(combat) and boolean(dead)
        and boolean(swimming) and boolean(flying) and boolean(taxi)
        and focusKnown and accessible(focus)
    local unavailable=loading or (accessible(connected) and connected==false) or (accessible(exists) and exists==false)
    local valid=statusKnown and not unavailable and number(x) and number(y) and number(c) and c>=0 and c<=65535
        and number(facing) and number(speed) and speed>=0 and speed<=65535/256
        and math.abs(x)<134217727 and math.abs(y)<134217727
    if valid then flags=flags+F.VALID end
    if boolean(combat) and combat then flags=flags+F.COMBAT end
    if boolean(dead) and dead then flags=flags+F.DEAD end
    if (boolean(swimming) and swimming) or yes(IsSubmerged) then flags=flags+F.SWIMMING end
    if boolean(flying) and flying then flags=flags+F.FLYING end
    if unavailable then flags=flags+F.LOADING end
    if focusKnown and accessible(focus) and focus then flags=flags+F.INPUT_FOCUS end
    if boolean(taxi) and taxi then flags=flags+F.TAXI end
    -- Some clients have no API for boats/lifts. Vehicle detection is not proof
    -- that every transport is detectable; the desktop additionally checks jumps.
    if yes(UnitInVehicle,'player') or yes(IsOnTransport) then flags=flags+F.TRANSPORT end
    seq=(seq+1)%4294967296
    local t={session=session,seq=seq,c=valid and math.floor(c) or 0,ui_map=math.floor(uiMap),
        x=valid and x or 0,y=valid and y or 0,z=valid and z or nil,
        facing=valid and facing or 0,speed=valid and speed or 0,flags=flags}
    local bytes=P.Encode(t)
    if not bytes then return end
    for index,byte in ipairs(bytes) do
        local high,low=math.floor(byte/16)*17,(byte%16)*17
        local hi=32+(index-1)*2+1
        cells[hi]:SetColorTexture(high/255,high/255,high/255,1)
        cells[hi+1]:SetColorTexture(low/255,low/255,low/255,1)
    end
    -- Clear padding when a v2 height sample changes back to a v1 fallback.
    for index=33+#bytes*2,112 do cells[index]:SetColorTexture(0,0,0,1) end
    AD.last=t
    AD.statusKnown=statusKnown
end
local function initialize()
    AutonomousDrivingDB=type(AutonomousDrivingDB)=='table' and AutonomousDrivingDB or {}
    local db=AutonomousDrivingDB
    db.generation=((number(db.generation) and math.floor(db.generation) or 0)+1)%65536
    session=(((time and time() or 1)%65536)*65536+math.floor((GetTime() or 0)*1000)%65536+db.generation)%4294967296
    db.session=session
    initialized=true
    layout()
    sample()
end
frame:RegisterEvent('ADDON_LOADED')
frame:RegisterEvent('PLAYER_ENTERING_WORLD')
frame:RegisterEvent('PLAYER_LEAVING_WORLD')
frame:SetScript('OnEvent',function(_,event,name)
    if event=='ADDON_LOADED' and name==addonName then initialize()
    elseif event=='PLAYER_ENTERING_WORLD' then loading=false
    elseif event=='PLAYER_LEAVING_WORLD' then loading=true end
    if initialized then sample() end
end)
frame:SetScript('OnUpdate',function(_,delta)
    if not initialized then return end
    elapsed=elapsed+delta
    if elapsed>=1/30 then elapsed=elapsed%(1/30);sample() end
end)
SLASH_AUTONOMOUSDRIVINGTELEMETRY1='/adgps'
SlashCmdList.AUTONOMOUSDRIVINGTELEMETRY=function(text)
    if not initialized then return end
    local command,a,b=(text or ''):match('^(%S*)%s*(%S*)%s*(%S*)')
    if command=='pos' and tonumber(a) and tonumber(b) then
        AutonomousDrivingDB.x=math.max(0,tonumber(a));AutonomousDrivingDB.y=math.max(0,tonumber(b));layout()
    elseif command=='cell' and tonumber(a) then AutonomousDrivingDB.cell=tonumber(a);layout()
    elseif command=='reset' then AutonomousDrivingDB.x=8;AutonomousDrivingDB.y=8;AutonomousDrivingDB.cell=8;layout()
    elseif command=='show' then frame:Show()
    elseif command=='hide' then frame:Hide()
    else
        print('AutonomousDriving: /adgps pos 8 8 | cell 8 | reset | show | hide')
        print('AD v2（含高度，兼容 v1）；仅输出遥测。船只和电梯检测取决于客户端公开 API，不保证完整。')
        if AD.last then print('session='..AD.last.session..' seq='..AD.last.seq..' flags='..AD.last.flags) end
        print('必要状态 API 可用='..tostring(AD.statusKnown))
    end
end
