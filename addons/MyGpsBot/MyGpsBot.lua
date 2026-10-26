local GpsFrame = CreateFrame("Frame", "MyGpsBotFrame", UIParent)
GpsFrame:SetSize(140, 50)
GpsFrame:SetPoint("TOPLEFT", UIParent, "TOPLEFT", 0, 0)

local bg = GpsFrame:CreateTexture(nil, "BACKGROUND")
bg:SetAllPoints(GpsFrame)
bg:SetColorTexture(0, 0, 0, 1) 
GpsFrame:Show()

local BLOCK_SIZE = 40

-- X 坐标方块（使用红通道）
local pixelX = GpsFrame:CreateTexture(nil, "OVERLAY")
pixelX:SetSize(BLOCK_SIZE, BLOCK_SIZE)
pixelX:SetPoint("TOPLEFT", GpsFrame, "TOPLEFT", 0, 0)
pixelX:Show()

-- Y 坐标方块（使用绿通道，换用另一个纹理防止批处理合并）
local pixelY = GpsFrame:CreateTexture(nil, "OVERLAY")
pixelY:SetSize(BLOCK_SIZE, BLOCK_SIZE)
pixelY:SetPoint("TOPLEFT", GpsFrame, "TOPLEFT", 45, 0)
pixelY:SetTexture("Interface\\Buttons\\UI-Quickslot2") -- 换用独立纹理
pixelY:Show()

-- 朝向方块（使用蓝通道，但由于我们用了独立的纹理且完全隔离，这次不会再串色了）
local pixelF = GpsFrame:CreateTexture(nil, "OVERLAY")
pixelF:SetSize(BLOCK_SIZE, BLOCK_SIZE)
pixelF:SetPoint("TOPLEFT", GpsFrame, "TOPLEFT", 90, 0)
pixelF:SetTexture("Interface\\Tooltips\\UI-Tooltip-Background") -- 换用另一独立纹理
pixelF:Show()

GpsFrame:SetScript("OnUpdate", function(self, elapsed)
    local mapID = C_Map.GetBestMapForUnit("player")
    if not mapID then return end
    
    local position = C_Map.GetPlayerMapPosition(mapID, "player")
    if position then
        local x, y = position:GetXY()
        local facing = GetPlayerFacing() or 0

        -- 分别独立设置，互不干扰
        pixelX:SetColorTexture(x, 0, 0, 1)                          -- X 独占红通道
        pixelY:SetColorTexture(0, y, 0, 1)                          -- Y 独占绿通道
        pixelF:SetColorTexture(0, 0, facing / 6.28318530718, 1)     -- 朝向独占蓝通道
    end
end)