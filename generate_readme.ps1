# BohleBots 2026 - README Generator
# Counts code lines from .cpp, .h and .tpp files and generates a dynamic README.md
# Compatible with PowerShell 5.1 (no literal emoji - uses Unicode code points)

$projectRoot = $PSScriptRoot

# --- Emoji variables (safe for PS 5.1) ---
$e_robot  = [char]::ConvertFromUtf32(0x1F916)
$e_chart  = [char]::ConvertFromUtf32(0x1F4CA)
$e_folder = [char]::ConvertFromUtf32(0x1F5C2)
$e_wrench = [char]::ConvertFromUtf32(0x1F6E0)
$e_box    = [char]::ConvertFromUtf32(0x1F4E6)
$e_file   = [char]::ConvertFromUtf32(0x1F4C1)
$e_check  = [char]::ConvertFromUtf32(0x2705)

# --- Collect all source files ---
$extensions = @("*.cpp", "*.h", "*.tpp")
$allFiles = @()
foreach ($ext in $extensions) {
    $allFiles += Get-ChildItem -Path $projectRoot -Recurse -Filter $ext |
        Where-Object { $_.FullName -notmatch '\\\.pio\\' -and $_.FullName -notmatch '\\\.idea\\' }
}

# --- Count lines per file ---
$fileData = @()
$totalLines = 0

foreach ($file in ($allFiles | Sort-Object FullName)) {
    $lines = (Get-Content $file.FullName -Encoding UTF8 -ErrorAction SilentlyContinue | Measure-Object -Line).Lines
    if ($null -eq $lines) { $lines = 0 }
    $totalLines += $lines
    $relativePath = $file.FullName.Replace($projectRoot + "\", "").Replace("\", "/")
    $fileData += [PSCustomObject]@{
        Path      = $relativePath
        Extension = $file.Extension
        Lines     = $lines
    }
}

# --- Group by extension ---
$byExt = $fileData | Group-Object Extension | Sort-Object Name
$extSummary = @()
foreach ($grp in $byExt) {
    $extLines = ($grp.Group | Measure-Object -Property Lines -Sum).Sum
    $extSummary += [PSCustomObject]@{
        Extension = $grp.Name
        Count     = $grp.Count
        Lines     = $extLines
    }
}

# --- Group by folder ---
$byFolder = $fileData | ForEach-Object {
    $parts = $_.Path -split "/"
    $folder = if ($parts.Count -gt 1) { ($parts[0..($parts.Count - 2)]) -join "/" } else { "." }
    [PSCustomObject]@{ Folder = $folder; Lines = $_.Lines }
} | Group-Object Folder | ForEach-Object {
    [PSCustomObject]@{
        Folder = $_.Name
        Lines  = ($_.Group | Measure-Object -Property Lines -Sum).Sum
    }
} | Sort-Object { -$_.Lines }

# --- Build tables ---
$fileTable = "| File | Lines |`r`n|------|-------|`r`n"
foreach ($f in $fileData) { $fileTable += "| ``$($f.Path)`` | $($f.Lines) |`r`n" }

$extTable = "| Extension | Files | Lines |`r`n|-----------|-------|-------|`r`n"
foreach ($e in $extSummary) { $extTable += "| ``$($e.Extension)`` | $($e.Count) | $($e.Lines) |`r`n" }

$folderTable = "| Folder | Lines |`r`n|--------|-------|`r`n"
foreach ($fo in $byFolder) { $folderTable += "| ``$($fo.Folder)`` | $($fo.Lines) |`r`n" }

# --- Git info ---
$gitBranch = ""; $gitCommit = ""; $gitDate = ""
try {
    $gitBranch = (git -C $projectRoot rev-parse --abbrev-ref HEAD 2>$null).Trim()
    $gitCommit = (git -C $projectRoot rev-parse --short HEAD 2>$null).Trim()
    $gitDate   = (git -C $projectRoot log -1 --format="%ci" 2>$null).Trim()
} catch {}

$gitSection = ""
if ($gitCommit) {
    $gitSection  = "## $e_box Repository Info`r`n`r`n"
    $gitSection += "| Property | Value |`r`n|----------|-------|`r`n"
    $gitSection += "| Branch | ``$gitBranch`` |`r`n"
    $gitSection += "| Last Commit | ``$gitCommit`` |`r`n"
    $gitSection += "| Commit Date | $gitDate |`r`n`r`n---`r`n`r`n"
}

# --- Timestamp ---
$timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
$nl = "`r`n"

# --- Assemble README (string concat, no here-string, avoids PS5.1 encoding issues) ---
$readme  = "# $e_robot BohleBots Pompeii 2026$nl$nl"
$readme += "> ESP32-based robot soccer project for the **RoboCup Junior** competition.  $nl"
$readme += "> Platform: **Arduino / ESP-IDF 5.3** on **ESP32**$nl$nl---$nl$nl"

$readme += "## $e_chart Code Statistics$nl$nl"
$readme += "> _Auto-generated on $timestamp_$nl$nl"
$readme += "**Total Lines of Code: $totalLines**$nl$nl"
$readme += "### By File Type$nl$nl$extTable$nl"
$readme += "### By Folder$nl$nl$folderTable$nl---$nl$nl"

$readme += "## $e_folder Source Files$nl$nl$fileTable$nl---$nl$nl"

$readme += "## $e_wrench Build Configuration$nl$nl"
$readme += "| Property | Value |$nl|----------|-------|$nl"
$readme += "| Platform | ESP32 (espressif32 / Arduino IDF5.3) |$nl"
$readme += "| Framework | Arduino |$nl"
$readme += "| Monitor Speed | 115200 baud |$nl"
$readme += "| Upload Speed | 921600 baud |$nl"
$readme += "| C++ Standard | C++20 (``-std=c++2a``) |$nl"
$readme += "| LTO | enabled (``-flto=4``) |$nl$nl"
$readme += "### Dependencies$nl$nl"
$readme += "| Library | Version |$nl|---------|---------|$nl"
$readme += "| elapsedMillis (pfeerick) | ^1.0.6 |$nl"
$readme += "| Wire | built-in |$nl"
$readme += "| PID (br3ttb) | ^1.2.1 |$nl$nl---$nl$nl"

$readme += $gitSection

$readme += "## $e_file Project Structure$nl$nl"
$readme += '```' + $nl
$readme += "bohlebots_2026/$nl"
$readme += "|- include/          # Header files$nl"
$readme += "|  |- comms/        # Communication (ESP-NOW, CM5)$nl"
$readme += "|  |- config/       # Configuration constants$nl"
$readme += "|  \- util/         # Utility classes (Vector2, MovingAverage, ...)$nl"
$readme += "|- src/              # Source files$nl"
$readme += "|  |- comms/$nl"
$readme += "|  \- util/$nl"
$readme += "|- lib/              # External libraries (PlatformIO)$nl"
$readme += "\- platformio.ini    # Build configuration$nl"
$readme += '```' + "$nl$nl---$nl$nl"
$readme += "_README generated by ``generate_readme.ps1``_$nl"

# --- Write UTF-8 without BOM ---
$readmePath = Join-Path $projectRoot "README.md"
$utf8NoBom = New-Object System.Text.UTF8Encoding $false
[System.IO.File]::WriteAllText($readmePath, $readme, $utf8NoBom)

Write-Host ""
Write-Host "$e_check  README.md generated successfully!" -ForegroundColor Green
Write-Host "   Total files : $($fileData.Count)" -ForegroundColor Cyan
Write-Host "   Total lines : $totalLines" -ForegroundColor Cyan
Write-Host "   Written to  : $readmePath" -ForegroundColor Cyan
Write-Host ""
