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
