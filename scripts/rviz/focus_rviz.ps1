Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
using System.Text;
using System.Collections.Generic;
public class Win32 {
    [DllImport("user32.dll")]
    public static extern bool EnumWindows(EnumWindowsProc enumProc, IntPtr lParam);
    public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);
    [DllImport("user32.dll")]
    public static extern int GetWindowText(IntPtr hWnd, StringBuilder lpString, int nMaxCount);
    [DllImport("user32.dll")]
    public static extern bool IsWindowVisible(IntPtr hWnd);
    [DllImport("user32.dll")]
    public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
    [DllImport("user32.dll")]
    public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll")]
    public static extern bool BringWindowToTop(IntPtr hWnd);
}
'@

$rvizHandle = [IntPtr]::Zero
$callback = {
    param($hWnd, $lParam)
    if ([Win32]::IsWindowVisible($hWnd)) {
        $title = New-Object System.Text.StringBuilder(256)
        [Win32]::GetWindowText($hWnd, $title, 256) | Out-Null
        if ($title.ToString() -match 'RViz') {
            $script:rvizHandle = $hWnd
        }
    }
    return $true
}
$delegate = [Win32+EnumWindowsProc]$callback
[Win32]::EnumWindows($delegate, [IntPtr]::Zero) | Out-Null

if ($rvizHandle -ne [IntPtr]::Zero) {
    [Win32]::ShowWindow($rvizHandle, 9) | Out-Null  # SW_RESTORE = 9
    Start-Sleep -Milliseconds 500
    [Win32]::SetForegroundWindow($rvizHandle) | Out-Null
    [Win32]::BringWindowToTop($rvizHandle) | Out-Null
    Write-Host "RViz2 window brought to front"
} else {
    Write-Host "RViz2 window not found"
}
