# Surface diagnostics v1

这是统一表面与加载图连通性接口的第一步代码补丁，不是某座桥的瓦片替换。

将本文件夹放到 D:\wangzekun\Documents\AutonomousDriving\tools\ 下。完整操作与代码说明见 surface_diagnostics_v1_guide.md。

PowerShell：

```powershell
& 'D:\wangzekun\Documents\AutonomousDriving\.venv\Scripts\python.exe' 'D:\wangzekun\Documents\AutonomousDriving\tools\surface_diagnostics_v1\install_surface_diagnostics.py'
```

安装器会在本机隔离编译，验证新接口和原步行 ABI，通过后更新源码并最后切换 manifest.library。
源码/包版本不匹配则停止；不会清空禁行边，不改变tiles身份。

本包没有预制Windows DLL。验证记录来自Linux原生构建、固定真实网格及合成文件事务；Windows验收由安装器在本机执行。
本次尚未接入UI或规划恢复逻辑，也未启用游泳。
