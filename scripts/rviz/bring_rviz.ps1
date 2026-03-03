Add-Type @"
using System;
using System.Runtime.InteropServices;
public class WinApi {
    [DllImport("user32.dll")]
    public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll")]
    public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
}
"@

$processes = Get-Process | Where-Object { $_.MainWindowHandle -ne 0 -and $_.MainWindowTitle -ne "" }
foreach ($p in $processes) {
    if ($p.MainWindowTitle -match "rviz|RViz" -or $p.ProcessName -match "rviz") {
        Write-Output "Found RViz2: $($p.MainWindowTitle)"
        [WinApi]::ShowWindow($p.MainWindowHandle, 9)
        Start-Sleep -Milliseconds 200
        [WinApi]::SetForegroundWindow($p.MainWindowHandle)
    }
}

# List all windows for debugging
$processes | Select-Object ProcessName, MainWindowTitle | Format-Table -AutoSize
