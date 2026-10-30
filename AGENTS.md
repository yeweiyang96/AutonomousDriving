# Autonomous Driving 项目约定

## 已有 Forever 服务端资源

用户明确要求：记住本机已有 Forever 服务端代码，可以利用其中的逻辑和数据。涉及地图、碰撞、寻路、坐标或游戏机制时，优先检查这些本地资源，复用经过核实的实现和数据，不要默认需要重新寻找或从零开发。

当前资源位于 WSL 的 Ubuntu 中（使用前验证路径与版本）：

- 集成服务端源码：`/home/wangzekun/forever/TrinityCore-integrated`
- 原始服务端源码：`/home/wangzekun/forever/TrinityCore`
- 已编译工具：`/home/wangzekun/forever/server-integrated/bin`
- 地形、碰撞和导航数据：`/home/wangzekun/forever/data-integrated/{maps,vmaps,mmaps}`
- 原始碰撞提取结果：`/home/wangzekun/forever/data-integrated/Buildings`
- 提取与构建日志：`/home/wangzekun/forever/logs`
- 当前服务端目录链接：`/home/wangzekun/forever/current`
- 本机游戏资源根目录：`C:\Program Files (x86)\World of Warcraft`，产品 `wowf`；版本从 `.build.info` 读取。

源码中重点参考 `src/tools/`、`src/common/mmaps_common/`、`src/server/game/Movement/`、碰撞模块与 `dep/recastnavigation/`。现有工具包括 `mapextractor`、`vmap4extractor`、`vmap4assembler`、`mmaps_generator`、`mmap_probe` 和道路诊断工具；实际可用参数以对应版本源码为准。

使用约定：

- 服务端源码、运行数据和日志可作为参考输入。新的提取、编译和实验结果默认写入本项目 `runtime/`，正式客户端数据写入本项目 `data/`，不覆盖正在运行的服务端数据。
- 记录复用源码的提交、必要的本地差异、许可和数据来源。旧数据不能仅因目录存在就被标注成当前客户端版本；核对格式、提取日志、覆盖范围和几何变化。
- 地形高度、静态碰撞、导航网格和游戏内动态状态是不同层次。离线数据验证不能代替实际游戏行走验收。
- 保留现有用户改动。复用资源不代表本任务授权修改服务端配置、数据库、运行状态或重新刷写设备。

## Navigation generation and release workflow

### General policy

Navigation problems must be handled as **systemic generation/planning issues**, not as one-off coordinate or tile patches.

Known historical failures included:

- bridge tops becoming disconnected after Recast span merging;
- valid ground surfaces being emitted as water after region partitioning;
- native Detour searches exhausting the 65,535-node pool when broken topology caused very large searches;
- road-guided planning accepting or attempting very large detours;
- mismatches between generated navmesh data and the native DLL ABI.

Do **not** respond to a newly discovered bad route by immediately adding an off-mesh link, changing one `.mmtile`, relaxing the agent radius, or allowing all water polygons.

First determine whether the failure belongs to an already known generation/planning class.

---

### Current validated global navmesh generation rule

The current global fix consists of **both** of the following changes.

#### 1. Recast higher-surface area preservation

File in the local Forever TrinityCore source:

```text
/home/wangzekun/forever/TrinityCore-integrated/dep/recastnavigation/Recast/Source/RecastRasterization.cpp
```

Inside `addSpan()`, the merge logic must preserve the area belonging to the physically higher surface when overlapping spans merge but their tops differ by more than `flagMergeThr`:

```cpp
// Merge flags.
// If the two tops are close, retain the original Recast preference rule.
// If they are farther apart but still form one merged span, keep the area
// belonging to the physically higher top surface.
if (rcAbs((int)s->smax - (int)cur->smax) <= flagMergeThr)
    s->area = rcMax(s->area, cur->area);
else if (cur->smax > s->smax)
    s->area = cur->area;
```

This fixes cases where a walkable upper surface such as a bridge deck was merged with lower/underside geometry and lost its `NAV_AREA_GROUND` classification.

#### 2. Monotone partitioning

All generated map 0 and map 1 navigation data must use:

```text
--partition monotone
```

The project records this explicitly as:

```python
SETTINGS['partition_method'] = 'monotone'
```

Do not silently fall back to `layers`, `auto`, or another partition method.

The combination of higher-surface area preservation **plus Monotone partitioning** is the validated global rule. Either fix alone was insufficient in previous bridge regressions.

---

### Validated reference cases

The global generation rule has reproduced the previously repaired bridge tiles byte-for-byte:

```text
Map 1 / tile 37,38
SHA256:
d8497ef8ac2cb510a2b950af4e24360d746cbe7f71699773c2ad3bcbd824f7ce
bytes:
1269700
```

```text
Map 0 / tile 50,34
SHA256:
74dd28d4d559518bca84b678ac3fb3ccd5ba294c8ab65bebcede927acc46df1c
bytes:
888768
```

These values are regression evidence for the known geometry/source set. Do not assume they remain identical after an intentional client-data or generator-version change.

---

### Generator binary

The AutonomousDriving-specific tool directory is:

```text
/home/wangzekun/forever/autonomous-driving-tools/bin
```

Do not overwrite or depend on:

```text
/home/wangzekun/forever/server-integrated/bin/mmaps_generator
```

for AutonomousDriving production generation.

The validated generator binary used for the current global rebuild had SHA256:

```text
c298fc3d04de4a2cd48d4a09bc53eca865f3155a66e69cbaca0b2678a6cb2de7
```

`tools/export_navigation.py` deliberately pins the expected generator SHA.

If the generator source is intentionally changed and rebuilt, do **not** simply update the expected SHA. First repeat the known bridge/regression validation, then update the pinned hash.

---

## Standard navigation rebuild workflow

### Step 1 — Build a candidate only

Never make a newly generated full map active immediately.

Run:

```powershell
& 'D:\wangzekun\Documents\AutonomousDriving\.venv\Scripts\python.exe' `
  'D:\wangzekun\Documents\AutonomousDriving\tools\export_navigation.py' `
  --prepare-only
```

Successful completion prints:

```text
NAVIGATION_CANDIDATE_READY
PREPARED = <runtime candidate directory>
```

The generated candidate remains under:

```text
runtime\nav-build-<client-version>-<timestamp>
```

and does not replace the active navigation manifest.

---

### Step 2 — Build a staging navigation pack

Use the prepared directory to create a non-production pack under `runtime`.

The pack must use the same current native DLL intended for production.

The exporter requires an explicit native library:

```text
--native-library <path-to-adrive_navmesh.dll>
```

Do not rely on an arbitrary `adrive_navmesh.dll` copied into an output root.

The selected DLL must expose at least:

```text
ad_path
ad_budget_reason
ad_path_bounded_v3
```

The DLL SHA and source path are recorded during publication.

---

### Step 3 — Run the permanent regression gate

The permanent regression checker is:

```text
tools\check_navigation_regressions.py
```

For the currently active pack:

```powershell
& 'D:\wangzekun\Documents\AutonomousDriving\.venv\Scripts\python.exe' `
  'D:\wangzekun\Documents\AutonomousDriving\tools\check_navigation_regressions.py'
```

For a staging/candidate pack:

```powershell
& 'D:\wangzekun\Documents\AutonomousDriving\.venv\Scripts\python.exe' `
  'D:\wangzekun\Documents\AutonomousDriving\tools\check_navigation_regressions.py' `
  --manifest '<candidate-root>\manifest.json'
```

The checker performs:

- complete manifest/hash validation;
- native loading of every navigation tile;
- `partition_method == monotone` verification;
- required native bounded-search ABI verification;
- Kalimdor bridge A→B and B→A with road guidance;
- Kalimdor bridge A→B and B→A direct native queries;
- Eastern Kingdoms bridge A→B and B→A;
- known long forward and reverse route regressions;
- Westfall recorded-route regression;
- Westfall route-vs-road quality checks;
- detection of unexpected final fallback behavior.

A candidate is acceptable only when the result ends with:

```text
=== SUMMARY ===
STATUS = passed
FAILURES = 0
NAVIGATION_REGRESSIONS_PASSED
```

Reports are stored under:

```text
runtime\navigation-regressions\
```

Do not publish a candidate when this checker fails.

---

### Step 4 — Publish the prepared candidate

After the candidate regression gate passes, publish the exact already-generated candidate.

Example:

```powershell
$ROOT = 'D:\wangzekun\Documents\AutonomousDriving'

& "$ROOT\.venv\Scripts\python.exe" `
  "$ROOT\tools\export_navigation.py" `
  --prepared '<prepared-directory>' `
  --native-library '<validated-adrive_navmesh.dll>' `
  --output "$ROOT\data\navigation"
```

Do not regenerate the maps between candidate validation and publication.

Publication performs another complete tile/hash/native-load validation before changing the active manifest.

---

### Step 5 — Re-run regression checks against the active pack

After publication, run:

```powershell
& 'D:\wangzekun\Documents\AutonomousDriving\.venv\Scripts\python.exe' `
  'D:\wangzekun\Documents\AutonomousDriving\tools\check_navigation_regressions.py'
```

Only consider the release complete after the **active** `data/navigation/manifest.json` also returns:

```text
STATUS = passed
FAILURES = 0
NAVIGATION_REGRESSIONS_PASSED
```

---

## Current validated production baseline

Validated active generation:

```text
client build:
1.60.1.70685

navigation identity:
7a4606dd9c967264

tile count:
1724

map 0:
736 tiles

map 1:
988 tiles

partition:
monotone
```

Validated native DLL SHA256:

```text
d7f994bbc6a59be18be466e5c880f1b4baeb1161ffa1b18cdb53bfa6c2ec3381
```

Known regression results for this generation:

```text
Kalimdor bridge guided A→B:
~49.6221 yd XY / ~49.6950 yd 3D

Kalimdor bridge guided B→A:
~49.6221 yd XY / ~49.6950 yd 3D

Eastern Kingdoms bridge A→B:
~40.8003 yd XY / ~41.0717 yd 3D

Eastern Kingdoms bridge B→A:
~40.8003 yd XY / ~41.0717 yd 3D

Known long route:
~3501.81 yd XY

Known long reverse:
~3514.39 yd XY
```

Westfall validated route:

```text
straight XY:
~2116.47 yd

reviewed-road guide:
~6763.95 yd

navmesh route:
~6674.80 yd

route / guide:
~0.9868

expanded:
9694

final fallbacks:
0

excessive_detour:
false
```

The long Westfall route is therefore expected: the reviewed road network itself is approximately 6.76k yards long. The ~6.67k-yard native route is not an abnormal Detour excursion.

---

## Search-budget rule

Do not treat Detour's 65,535-node pool as a parameter that should simply be increased.

A node-pool exhaustion event can indicate broken topology or an unexpectedly large search domain.

The project now uses bounded native road searches through:

```text
ad_path_bounded_v3
```

Failures such as:

```text
road_scope_unconnected
road_connection_unverified
```

must be investigated as topology/road-reference/data issues before changing search budgets.

`global_unreachable_proven=false` means only that the bounded loaded search domain did not establish a valid connection. It is not proof that no route exists anywhere on the map.

---

## Water and swimming policy

Current production navigation remains:

```text
water = false
```

Walking queries must not solve topology errors by treating arbitrary water polygons as walkable ground.

Do not change the normal ground filter to include water as a shortcut.

Swimming is a separate future mobility mode and should be implemented explicitly as something equivalent to:

```text
GROUND
  -> ENTER_WATER
  -> SWIM
  -> EXIT_WATER
  -> GROUND
```

Both water entry and exit must be validated.

Swimming support must not be used to hide a bridge surface-classification or navmesh-generation bug.

---

## Historical repair data

Directories such as:

```text
data/navigation/repairs/
data/navigation/route-search-bounds/
runtime/manifest-before-global-nav-70685.json
```

may remain as rollback/diagnostic evidence.

The current production navmesh should **not** depend on the old per-tile bridge repairs. Those repairs have been superseded by the global Recast + Monotone generation rule.

Do not delete historical repair/rollback material automatically.

---

## Rule for future navigation failures

When a new navigation failure is reported:

1. reproduce it using the current active manifest;
2. preserve the exact start, goal, guide, navigation identity, DLL SHA, and planner diagnostics;
3. determine whether the problem is:
   - endpoint surface selection;
   - span/area generation;
   - partition/region generation;
   - cross-tile topology;
   - road-corridor quality;
   - bounded native search;
   - missing mobility mode such as swimming;
4. compare against existing regression classes;
5. implement a geometry/rule-level fix where possible;
6. add the failure as a permanent regression case;
7. regenerate a candidate;
8. run `check_navigation_regressions.py`;
9. publish only after the candidate and active-pack regression gates both pass.

Avoid coordinate-specific production patches unless the underlying source geometry itself is genuinely exceptional and cannot be represented by a general rule
