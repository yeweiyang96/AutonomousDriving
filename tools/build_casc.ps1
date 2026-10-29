param([string]$CMake)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$revision = '6b889ec8f421cbc9baccec296c582362ab917887'
$work = Join-Path $projectRoot 'runtime\map-tools'
New-Item -ItemType Directory -Force -Path $work | Out-Null
if (-not $CMake) {
    $found = Get-Command cmake -ErrorAction SilentlyContinue
    if ($found) { $CMake = $found.Source }
    else {
        $bundled = Join-Path (Split-Path -Parent $projectRoot) 'hid_usb\.tools\cmake'
        $CMake = Get-ChildItem -LiteralPath $bundled -Filter cmake.exe -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty FullName
    }
}
if (-not $CMake) { throw '请安装 CMake，或通过 -CMake 指定 cmake.exe。还需要 Visual Studio C++ Build Tools。' }
$archive = Join-Path $work "CascLib-$revision.zip"
$source = Join-Path $work "CascLib-$revision"
if (-not (Test-Path -LiteralPath $source)) {
    Invoke-WebRequest -Uri "https://codeload.github.com/ladislav-zezula/CascLib/zip/$revision" -OutFile $archive
    Expand-Archive -LiteralPath $archive -DestinationPath $work
}
$build = Join-Path $work 'build'
& $CMake -S $source -B $build -A x64 '-DCMAKE_POLICY_VERSION_MINIMUM=3.5' '-DCASC_UNICODE=ON' '-DCASC_BUILD_SHARED_LIB=ON' '-DCASC_BUILD_STATIC_LIB=OFF' '-DCASC_BUILD_TESTS=OFF'
if ($LASTEXITCODE -ne 0) { throw 'CascLib 配置失败。' }
& $CMake --build $build --config Release --parallel 4
if ($LASTEXITCODE -ne 0) { throw 'CascLib 编译失败。' }
$dll = Join-Path $build 'Release\CascLib.dll'
$metadata = [ordered]@{
    repository = 'https://github.com/ladislav-zezula/CascLib'
    revision = $revision
    archive_sha256 = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLowerInvariant()
    dll_sha256 = (Get-FileHash -LiteralPath $dll -Algorithm SHA256).Hash.ToLowerInvariant()
    unicode = $true
}
$metadata | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $work 'source.json') -Encoding utf8
Write-Output $dll
