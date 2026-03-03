Add-Type @"
using System;
using System.Text;
using System.Collections.Generic;
using System.Runtime.InteropServices;
public class WindowEnum {
    public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);
    [DllImport("user32.dll")]
    public static extern bool EnumWindows(EnumWindowsProc lpEnumFunc, IntPtr lParam);
    [DllImport("user32.dll")]
    public static extern int GetWindowText(IntPtr hWnd, StringBuilder lpString, int nMaxCount);
    [DllImport("user32.dll")]
    public static extern bool IsWindowVisible(IntPtr hWnd);
    [DllImport("user32.dll")]
    public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll")]
    public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);

    public static List<Tuple<IntPtr, string>> GetAllWindows() {
        var windows = new List<Tuple<IntPtr, string>>();
        EnumWindows((hwnd, lParam) => {
            if (IsWindowVisible(hwnd)) {
                var sb = new StringBuilder(256);
                GetWindowText(hwnd, sb, 256);
                string title = sb.ToString();
                if (title.Length > 0) {
                    windows.Add(new Tuple<IntPtr, string>(hwnd, title));
                }
            }
            return true;
        }, IntPtr.Zero);
        return windows;
    }
}
"@

$windows = [WindowEnum]::GetAllWindows()
foreach ($w in $windows) {
    Write-Output "Handle: $($w.Item1) | Title: $($w.Item2)"
    # Bring RViz2 to front
    if ($w.Item2 -match "rviz|RViz") {
        Write-Output "  -> Found RViz2! Bringing to front..."
        [WindowEnum]::ShowWindow($w.Item1, 3)
        Start-Sleep -Milliseconds 300
        [WindowEnum]::SetForegroundWindow($w.Item1)
    }
}
