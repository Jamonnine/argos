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
}
'@

$windows = New-Object System.Collections.Generic.List[string]
$callback = {
    param($hWnd, $lParam)
    if ([Win32]::IsWindowVisible($hWnd)) {
        $title = New-Object System.Text.StringBuilder(256)
        [Win32]::GetWindowText($hWnd, $title, 256) | Out-Null
        if ($title.Length -gt 0) { $windows.Add($title.ToString()) }
    }
    return $true
}
$delegate = [Win32+EnumWindowsProc]$callback
[Win32]::EnumWindows($delegate, [IntPtr]::Zero) | Out-Null

$windows | Where-Object { $_ -match 'RViz|Gazebo|rviz|gazebo|Ubuntu|ROS|Ignition' }
