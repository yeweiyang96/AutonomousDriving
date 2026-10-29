param([switch]$Demo, [string]$Replay, [switch]$Test, [switch]$Extract)
$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot
$taskPython = Join-Path $PSScriptRoot '.venv\Scripts\python.exe'
$taskTemp = Join-Path $PSScriptRoot 'runtime\tmp'
New-Item -ItemType Directory -Path $taskTemp -Force | Out-Null
$env:TEMP = $taskTemp
$env:TMP = $taskTemp
if (-not (Test-Path -LiteralPath $taskPython)) {
    & python -m venv .venv
    if ($LASTEXITCODE -ne 0) { throw 'Python 环境创建失败，请先安装 Python 3.15。' }
}
& $taskPython -c "import sys; sys.exit(0 if sys.version_info[:2] == (3, 15) else 1)"
if ($LASTEXITCODE -ne 0) { throw '需要 Python 3.15。请使用正确的 Python 重新创建本项目 .venv。' }
& $taskPython -c "import importlib.util,sys; sys.exit(0 if importlib.util.find_spec('PIL') and __import__('PIL').__version__ == '13.0.0' else 1)"
if ($LASTEXITCODE -ne 0) {
    & $taskPython -m ensurepip --upgrade
    & $taskPython -m pip install --disable-pip-version-check -r requirements.txt
    if ($LASTEXITCODE -ne 0) { throw '依赖安装失败，请检查网络后重试。' }
}
if ($Extract -or -not (Test-Path -LiteralPath 'data\bundle.json')) {
    & $taskPython tools/extract_data.py
    if ($LASTEXITCODE -ne 0) { throw '参考数据提取失败。' }
}
if ($Test) {
    & $taskPython -m unittest discover -s tests -v
    exit $LASTEXITCODE
}
$taskArguments = @('-m', 'adrive')
if ($Demo) { $taskArguments += '--demo' }
if ($Replay) { $taskArguments += @('--replay', $Replay) }
& $taskPython @taskArguments
exit $LASTEXITCODE
