# AD v1 色块遥测协议

本协议连接独立插件 `AutonomousDriving` 与本机客户端。插件只读取公开游戏 API 并显示色块，不执行移动、点击、宏或网络请求。不检查游戏版本/build。协议版本用于保证双方解码一致，与游戏版本无关。

## 安装与观察

1. 将 `addons/AutonomousDriving` 文件夹复制到游戏的 `Interface/AddOns/`，进入游戏后启用。
2. 停用旧的 MyGpsBot 遥测插件以避免左上角色块重叠；MapWise/OpenMap 无需在游戏中加载。
3. 默认色块位于游戏 UI 左上角 `(8, 8)`，大小为 `128 × 64` UI 像素。
4. `/adgps` 显示会话、帧号、状态位和必要 API 的可用状态。`/adgps pos 8 8` 移动位置，`/adgps cell 8` 设置单格大小（4–24 UI 像素），`/adgps reset` 恢复位置/尺寸。`hide` 会令客户端遥测过期，`show` 重新显示。

游戏必须使用可见窗口或无边框窗口，色块不能被其他窗口挡住。客户端适配窗口位置和 DPI，不更改系统缩放设置。解码器支持单格约 3–64 物理像素；建议使用默认大小。HDR、滤镜或色彩设置使色阶不可区分时会拒绝解码，应先恢复可辨识色块再开始校准。

## 二进制帧

客户端兼容 v1（34 字节）和 v2（38 字节），整数均小端。坐标是世界码数：`x` 增大向北、`y` 增大向西、`z` 为高度。大陆编号 `c` 与 `uiMapID` 不可混用；道路/地形只覆盖 `c=0/1`。

v2 在下表偏移 32 处增加 `int32 z16`（世界高度 × 16），CRC 移至偏移 36，覆盖前 36 字节，version 为 2；其余字段不变。v1 保留原布局，解码后的高度为未知 `None`，不能当作零高度。游戏插件在 `UnitPosition` 提供有效高度时输出 v2；地图坐标转换备用路径没有高度时输出 v1，并清空旧帧遗留的填充格。

| 字节偏移 | 类型 | 字段 | 含义 |
| --- | --- | --- | --- |
| 0 | 2 bytes | magic | ASCII `AD` |
| 2 | uint8 | version | 固定 `1` |
| 3 | uint8 | reserved | 固定 `0` |
| 4 | uint32 | session | 本次插件会话，重载后改变 |
| 8 | uint32 | seq | 每次采样递增，按模 `2^32` 回绕 |
| 12 | uint16 | c | 世界大陆/实例编号 |
| 14 | uint32 | ui_map | 当前 UI 地图编号；不可获取时为 `0` |
| 18 | int32 | x16 | 世界北向坐标 × 16 |
| 22 | int32 | y16 | 世界西向坐标 × 16 |
| 26 | uint16 | facing | 朝向 / `2π` × 65536，回绕取模 |
| 28 | uint16 | speed256 | 当前速度（码/秒）× 256 |
| 30 | uint16 | flags | 下表状态位 |
| 32 | uint16 | crc | 前 32 字节的 CRC-16/CCITT-FALSE |

CRC 参数：poly=`0x1021`，init=`0xFFFF`，refin/refout=false，xorout=`0`；存储小端。坐标和速度取整统一使用 `floor(value + 0.5)`，包括负数，禁止 Python 的 ties-to-even 取整混入协议。

| flags | 含义 |
| --- | --- |
| `1` | 世界位置、朝向、速度及必要状态读取有效 |
| `2` | 战斗 |
| `4` | 死亡或灵魂状态 |
| `8` | 游泳或潜水 |
| `16` | 飞行 |
| `32` | 加载、离开世界或已断开 |
| `64` | 键盘焦点在游戏输入控件中 |
| `128` | 正在乘坐鸟点交通 |
| `256` | 公开 API 报告载具或运输状态 |

朝向 `0` 指向世界北方，正角度转向西方。编码精度为坐标 1/16 码、朝向一圈的 1/65536、速度 1/256 码/秒。

## 色块布局

面板为 16 列 × 8 行，按从左到右、从上到下排列；没有透明度或单格间隔。第一行（行号 0）是定位标记，第 7 行为其倒序：

`品红、青、黄、黑、白、红、绿、蓝、白、黑、青、品红、蓝、黄、红、绿`

颜色使用通道值 0 或 255。第 1 行为 16 个灰度级：`(17*i, 17*i, 17*i)`，`i=0..15`。

第 2–6 行承载帧：每字节先高半字节、再低半字节，每半字节用对应灰度级显示。v1 占 68 格、余下 12 格填黑；v2 占 76 格、余下 4 格填黑。面板仍是 128×64 UI 像素，无需改变截图尺寸或显示设置。解码时同时验证首尾定位行、灰度单调性、数据色阶置信度、黑色填充和 CRC，不把坏帧当作零坐标。

精细碰撞导航要求实时高度，以免把桥下角色当作已经到达桥上路线。使用旧 v1 插件仍可读取位置和查看地图，但执行精细路线前必须更新本项目游戏插件并 `/reload`。这项更新不需要修改 Pico 固件。

## 有效性与时间

插件目标更新频率为 30 Hz，受实际游戏帧率限制。主路径使用 `UnitPosition('player')`，缺失时才通过 `C_Map` 把当前地图位置转换为世界坐标；不会使用归一化地图坐标代替世界坐标。

必要的连接、角色存在、战斗、死亡、游泳、飞行、鸟点和输入焦点 API 不可用或结果不可访问时，会清除 `VALID`。位置/朝向/速度无效或加载时持续发送无效帧，并将坐标、朝向、速度字段置零，不重复旧的有效位置。游戏加载期间若不渲染画面，客户端通过帧号冻结识别过期。

`FreshnessTracker` 只接受向前的帧号：`0 < ((new-old) & 0xFFFFFFFF) < 0x80000000`。重复、倒序帧不更新时间。会话改变后旧会话退役，控制层必须取消动作并要求用户继续。截图的采集时刻会一直保留，耗时定位或解码不会让旧截图重新变新。250 ms 没有新有效采集帧时停止控制；新的无效状态帧也会立即触发控制层停止。

运输检测使用可用的 `UnitInVehicle` / `IsOnTransport`，不保证能识别所有船只或电梯；首版不规划这些交通方式，不能把 `TRANSPORT=0` 当作已证明处于普通地面。真实客户端的这些 API 和像素输出必须单独验收。

## 客户端捕获

`WindowsCapture(window_title='World of Warcraft', hwnd=None)` 通过精确窗口标题或明确选择的窗口句柄绑定游戏。没有匹配、存在多个同名窗口或窗口已最小化时不输出遥测。绑定后核对窗口所属进程和标题，窗口句柄被其他程序复用时停止识别。

初次定位优先检查客户区左上部，必要时扫描完整客户区。定位后只用 Win32 `BitBlt` 抓取面板矩形；窗口移动时更新屏幕原点，尺寸变化或连续解码失败后重新定位。坏帧返回 `None`，错误原因在 `status`。`foreground()` 只有已绑定的同一窗口位于前台才返回真。

图像解码与数据模型可跨平台测试；实时抓屏仅支持 Windows。

## 固定校验样例

输入：session=`0x12345678`，seq=`0xFFFFFFFE`，c=`1`，ui_map=`1411`，x=`-1234.5`，y=`5678.25`，facing=`π/2`，speed=`7`，flags=`1`。

预期 34 字节十六进制：

```text
4144010078563412feffffff010083050000d8b2ffffe4620100004000070100066c
```

离线测试包含这个固定样例、负坐标、角度回绕、全部字节单比特损坏、非整数缩放、灰度变化、遮挡和帧新鲜度。合成测试不能代替游戏内插件加载、实际色彩输出和端到端延迟验收。

## 执行 Lua 插件的离线测试

可选开发依赖与生产依赖分开，`requirements-dev.txt` 固定 `lupa==2.8`：

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
.\.venv\Scripts\python.exe -m unittest tests.test_addon_lua -v
```

测试使用 Lupa 的 **Lua 5.1** 运行器（没有该模块时尝试 LuaJIT 2.1），真实执行生产插件文件。WoW API 和 Frame/Texture 用内存 stub 提供；测试从 Lua 设置出的纹理颜色重建面板，再调用生产 Python 解码器。不存在真实游戏输入、网络通信或键鼠动作。缺少可选运行器时测试明确 `skip`，不能把跳过视为 Lua 验证通过。

本机 Python 3.15 暂无匹配的 Lupa 2.8 预编译包。若普通安装遇到 Windows 编译长路径错误，可在已安装 MSVC C/C++ 构建工具的环境中改用项目短路径：

```powershell
New-Item -ItemType Directory -Force runtime\lua-test, runtime\tmp | Out-Null
$env:TEMP = (Resolve-Path runtime\tmp).Path
$env:TMP = $env:TEMP
$env:LUPA_NO_LUAJIT = 'true'
.\.venv\Scripts\python.exe -m pip download --no-deps --no-binary=lupa lupa==2.8 -d runtime\lua-test
.\.venv\Scripts\python.exe -c "import tarfile; tarfile.open('runtime/lua-test/lupa-2.8.tar.gz').extractall('runtime/lua-test', filter='data')"
.\.venv\Scripts\python.exe -m pip install .\runtime\lua-test\lupa-2.8
.\.venv\Scripts\python.exe -m unittest tests.test_addon_lua -v
```

这里的 `LUPA_NO_LUAJIT` 只禁用不需要的 LuaJIT 源码构建，Lua 5.1 仍会编译。源代码压缩包 SHA256 为 `d8022641b9ec8ecf2c5ecbe9f47e5a70e0b87c4b5ae921b92cb02a638e0acd08`。构建中间文件留在被忽略的 `runtime/`，不改变生产依赖或系统 Python。

2026-10-29 已在本机 `.venv` 的 Python 3.15 + Lupa 2.8/Lua 5.1 下执行 11 项测试并全部通过，没有跳过。覆盖插件加载、面板渲染、编码金样、负坐标/角度取整、采样频率、战斗/死亡/游泳/飞行/鸟点/载具、加载/掉线、输入焦点、API 缺失及重载。焦点 API 抛错会清除有效状态，不会与正常返回 `nil` 的无焦点状态混淆。此结果仅证明离线 API stub 下的代码行为，不证明真实 Forever 客户端 API、画面或行走效果。
