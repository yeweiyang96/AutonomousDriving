# 第一步代码修改：统一表面与连通性诊断接口

日期：2026-10-29。适用基线：当前已安装原生预算诊断与道路规划修复的 AutonomousDriving 项目。

这一步新增一套可以复用的只读接口，让代码能查询“同一坐标下有哪些表面、标签是什么、当前规则是否允许、两个表面在已加载图中是否相连”。它服务后续公共网格验收、分层寻路和游泳，不包含任何桥坐标特例。

**本包已经实现代码与验证。它尚未启用游泳，也未接入 UI 或改变现有道路规划策略，因此安装后原来的失败路线不会仅凭这一步自动恢复。** 后续规划器可以直接复用这组接口，无需重新编写每座桥的采集程序。

## 现在只做这一项：安装接口补丁

将 ZIP 内的 `surface_diagnostics_v1` 文件夹解压到：

```text
D:\wangzekun\Documents\AutonomousDriving\tools\
```

确认安装器路径是：

```text
D:\wangzekun\Documents\AutonomousDriving\tools\surface_diagnostics_v1\install_surface_diagnostics.py
```

在 PowerShell 运行：

```powershell
& 'D:\wangzekun\Documents\AutonomousDriving\.venv\Scripts\python.exe' 'D:\wangzekun\Documents\AutonomousDriving\tools\surface_diagnostics_v1\install_surface_diagnostics.py'
```

脚本调用 Python 和 CMake，不需要执行 `.ps1`。构建过程每隔约十秒显示状态，不会让终端长时间没有说明。需要项目现有的 x64 C++ 构建环境；工具链自动发现逻辑沿用已有更新工具，不固定要求某一代 Visual Studio。

成功标志是：

```text
SURFACE_DIAGNOSTICS_INSTALLED
NEW_DLL = ...\data\navigation\diagnostics\...\adrive_navmesh.dll
NAVIGATION_IDENTITY_UNCHANGED = ...
REPORT = ...\runtime\surface-diagnostics-v1\...\report.json
```

看到成功后重新启动 AutonomousDriving，使它读取新 DLL。安装器不会结束正在运行的程序。

若版本检查不通过，会显示 `UPDATE_NOT_STARTED` 或 `UPDATE_FAILED`，应保留该输出；不能通过删除校验条件强行覆盖。重复运行已安装包也会因现有源码或新增文件不同而拒绝覆盖。本包没有猜测用户后来可能做过的修改。

## 实际改动哪些代码

| 文件 | 改动 | 用途 |
|---|---|---|
| `native/navmesh/bridge.cpp` | 在文件末尾追加一个 include | 将新接口编译进同一个 DLL |
| `native/navmesh/surface_diagnostics_v2.h` | 新增 | 固定宽度 C ABI 结构和枚举 |
| `native/navmesh/surface_diagnostics_v2.inc` | 新增 | 表面枚举与有界图连通性查询 |
| `adrive/nav_diagnostics.py` | 新增 | ctypes 绑定、能力检查和结构化结果 |
| `data/navigation/manifest.json` | 只更新 `library.path/bytes/sha256` | 保证程序实际加载已验证的新 DLL |

对现有 `bridge.cpp` 的改动就是：

```cpp
#include "surface_diagnostics_v2.inc"
```

两个新增 C++ 文件位于包内 `payload/`，完整实现可直接阅读。原 `ad_path`、`GroundFilter` 和旧 surface resolver 保留原代码；`adrive/navmesh.py` 与 `road_corridor.py` 不被旧版本文件覆盖。

安装器在新的构建目录中加入专用测试目标，生产 `CMakeLists.txt` 无需改动。编译后的 DLL 放在新的版本目录，避免覆盖正在被进程占用的旧 DLL。

## 新接口如何工作

### `ad_capabilities_v2`

返回 ABI 版本、结构大小、64 位 polygon ref、已实现能力和现有步行过滤掩码。Python 会先核对这些值，缺少导出或结构不匹配时直接报错，不会把旧 DLL 误当成支持新功能。

这一版的结构大小为：Capabilities 48 B、Surface 72 B、SurfaceSummary 48 B、ReachabilityRequest 40 B、ReachabilityReport 80 B。它们在 C++ 中有 `static_assert`，Python 端也核验 `ctypes.sizeof`。

### `ad_query_surfaces_v2`

输入原始 `(north, west, height)`，未知高度用 NaN。Python wrapper 支持用 `None` 表示未知高度。

每个候选返回：

```text
poly_ref（只在当前 native context 有效）
tile_x / tile_y / tile_layer / poly_index
north / west / height
area / flags / poly_type
walk_filter_pass / blocked_self
semantic_evidence = NAV_TAG_ONLY
```

枚举直接遍历已加载的 tile，保留普通 polygon 的所有类别，包括 water、被排除类别和 flags=0。这样既不会被旧步行 filter 隐藏，也不会受 Detour `queryPolygons` 内部最多32个邻层枚举的影响。

候选按 ref 保持独立：同 XY 多层、同高不同类别不会因为高度接近而合并。只有原始 XY 确实位于该 polygon 内才返回，不吸附到其他位置。

输出容量最多65535条；内部暂存不超过调用者给出的容量。容量不足时保留真实总数并显式标记截断，截断子集不能当作完整列表或全局最低的若干层。容量为0时只计数，不保存记录。

`area=9 / flags=4` 只说明网格把它标成 water。wrapper 明确保留 `physical_kind=UNKNOWN`，不把标签当成真实水体证明，更不因此允许游泳。

### `ad_query_reachability_v2`

输入来自同一 native context 的两个 ref，沿当前网格的真实有向 links 检查是否相连。两个 profile 分别为：

- `walk`：使用当前步行过滤和禁行记录。
- `tag_agnostic`：忽略类别，仅用于比较图结构；仍排除 offmesh 并保留禁行规则。结果不是步行路线或游泳路线。

访问记录容量为1到1000000，边检查上限为1到32000000，使用独立临时状态，不占用旧 `ad_path` 的65535节点池，也不重置上一次原生预算原因。每个 context 的加载与查询仍须串行调用。

结果严格区分：

| `result_name` | 含义 |
|---|---|
| `reachable_in_loaded_graph` | 当前已加载图中有符合本次规则的图路径 |
| `disconnected_in_loaded_graph` | 在当前已加载图中遍历完毕后没有到达目标 |
| `unknown_resource_limit` | 访问容量或边检查数到限，尚未判定 |
| `cancelled` | 已取消，尚未判定 |
| `endpoint_excluded` | 端点不符合过滤或禁行条件 |
| `invalid_reference` / `invalid_graph_link` | 引用失效或图数据异常 |

原生接口不知道 manifest 中还有哪些瓦片未加载，因此所有结果都标为 `loaded_graph_only`，不会宣布整个大陆不可达。可达结果也不包含几何路线、不保证路线短或满足道路偏好。

## 后续在现有 Mesh 中怎样调用

下面是给开发者的接口示例，前提是已经取得当前项目的 `Mesh` 对象，并通过它加载了声明范围内的瓦片。它不是本次需要另外执行的命令。

```python
from adrive.nav_diagnostics import Diagnostics

# mesh 是已创建、未关闭的 adrive.navmesh.Mesh。
diagnostics = Diagnostics(mesh.pack.native.dll)

surface_report = diagnostics.surfaces(
    mesh.handle,
    (point.x, point.y, point.z),
    capacity=256,
)

# 检查 complete_for_loaded_graph 后再使用完整候选集合。
# start_ref / goal_ref 必须来自同一个 mesh.handle，不能跨上下文复用。
connection_report = diagnostics.reachability(
    mesh.handle,
    start_ref,
    goal_ref,
    profile="walk",
    max_visits=100000,
    max_link_checks=2000000,
)
```

调用者后续应同时记录导航包身份、请求/加载/缺失瓦片集合、阻断版本与实际 tile 摘要。持久化 surface 身份不能只保存 native ref，至少要绑定 tile 内容、层与 polygon。

## 编译、安装和原有数据的关系

安装器先核对已捕获的 bridge/CMake 基线，容许纯 UTF-8 BOM 或 CRLF 差异，精确备份保留原始字节。源文件或 manifest 在构建期间变化时会拒绝继续提交。

Windows 的 `path.stat` 与 `os.fstat` 分别和自身的前后读数比较，避免再次出现之前“所有文件都被误报读取期间变化”的问题。

编译成功后，在独立子进程加载候选 DLL，跑新接口与旧 ABI 对比。通过后才写入新文件、追加 include，并最后切换 `manifest.library`。旧 DLL 保留，备份和过程记录位于：

```text
D:\wangzekun\Documents\AutonomousDriving\runtime\surface-diagnostics-v1\<本次编号>\
```

多文件写入不是一个原子事务；失败时只回滚仍等于本次写入内容的文件，不覆盖用户后来修改。异常若发生在写入之后，报告会明确列出恢复状态。

本次不改变瓦片内容或 `tiles` 清单，现有导航 identity 由 `tiles` 计算，因此保持不变。**原来绑定该版本的禁行边记录继续适用。** 这与之前改变桥瓦片 polygon 编号的更新不同。

## 已完成的验证，以及验证范围

本地 Linux 下，已用固定 Detour 源码编译实际动态库和测试网格生成器。13组原生接口检查通过，包括：同 XY 多层、同高不同类别、water/flags0、孔洞不吸附、33层完整枚举、容量截断、双向禁行、自环禁行、offmesh 排除、取消、ABI错误、旧路径点列/refs/展开一致、旧预算原因不被诊断覆盖。

另外在既有 Westfall 真实46瓦片、50608744 B 的固定证据上验证：新接口返回了约21.44、31.52、40.15码三个表面，其中中间层的实际标签为 water；遍历133157个地面 polygon 后，正确报告上层孤岛在该加载图中不可达。另一对表面在同一图中可达。降低诊断容量时返回“未判定”，不会误写成不可达。这是接口正确性的验证，未修复该区域的生成数据。

安装器另有8项文件稳定性与条件回滚检查。这些是模拟文件事务测试，不能当作已经在用户 Windows 上安装成功。用户运行上面的单条命令时，会在本机编译，并用当前实际 DLL 作为旧 ABI 对照重新验证；通过后才安装。

包内 `validation/` 保存验证记录。没有包含 Linux `.so` 作为 Windows DLL，也没有预制未经本机验证的 Windows DLL。
