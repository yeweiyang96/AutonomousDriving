-- AD telemetry v1/v2. v2 adds height while preserving the panel dimensions.
local _, AD = ...
local P = {VERSION=1, HEIGHT_VERSION=2, COLS=16, ROWS=8, FRAME_BYTES=34, HEIGHT_FRAME_BYTES=38}
AD.Protocol = P
local floor, pi = math.floor, math.pi
local nativeXor = (bit and bit.bxor) or (bit32 and bit32.bxor)
local function xor(a,b)
    if nativeXor then return nativeXor(a,b) end
    local value, place = 0, 1
    for _=1,16 do
        local aa,bb=a%2,b%2
        if aa~=bb then value=value+place end
        a=floor(a/2); b=floor(b/2); place=place*2
    end
    return value
end
local function add(bytes, value, count)
    if value<0 then value=value+4294967296 end
    for _=1,count do
        bytes[#bytes+1]=value%256
        value=floor(value/256)
    end
end
local function finite(n)
    return type(n)=='number' and n==n and n~=math.huge and n~=-math.huge
end
local function integer(n,lo,hi)
    return finite(n) and n==floor(n) and n>=lo and n<=hi
end
function P.Encode(t)
    if not (integer(t.session,0,4294967295) and integer(t.seq,0,4294967295)
      and integer(t.c,0,65535) and integer(t.ui_map,0,4294967295)
      and integer(t.flags,0,511) and finite(t.x) and finite(t.y)
      and finite(t.facing) and finite(t.speed) and t.speed>=0 and t.speed<=65535/256) then
        return nil,'invalid telemetry'
    end
    local x,y=floor(t.x*16+.5),floor(t.y*16+.5)
    if x < -2147483648 or x>2147483647 or y < -2147483648 or y>2147483647 then return nil,'coordinate overflow' end
    if t.z~=nil and (not finite(t.z) or t.z < -134217728 or t.z >= 134217727) then return nil,'height overflow' end
    local bytes={65,68,t.z~=nil and 2 or 1,0}
    add(bytes,t.session,4); add(bytes,t.seq,4); add(bytes,t.c,2); add(bytes,t.ui_map,4)
    add(bytes,x,4); add(bytes,y,4)
    add(bytes,floor((t.facing%(2*pi))/(2*pi)*65536+.5)%65536,2)
    add(bytes,floor(t.speed*256+.5),2); add(bytes,t.flags,2)
    if t.z~=nil then add(bytes,floor(t.z*16+.5),4) end
    local crc=65535
    for _,byte in ipairs(bytes) do
        crc=xor(crc,byte*256)
        for _=1,8 do
            local high=crc>=32768
            crc=(crc*2)%65536
            if high then crc=xor(crc,4129) end
        end
    end
    add(bytes,crc,2)
    return bytes
end
P.Markers={
    {255,0,255},{0,255,255},{255,255,0},{0,0,0},
    {255,255,255},{255,0,0},{0,255,0},{0,0,255},
    {255,255,255},{0,0,0},{0,255,255},{255,0,255},
    {0,0,255},{255,255,0},{255,0,0},{0,255,0},
}
return P
