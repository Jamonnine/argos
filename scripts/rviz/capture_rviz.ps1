Add-Type @"
using System;
using System.Runtime.InteropServices;
using System.Drawing;
using System.Drawing.Imaging;
public class ScreenCapture {
    [DllImport("user32.dll")]
    public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll")]
    public static extern bool ShowWindow(IntPtr hWnd, int nCmdShow);
    [DllImport("user32.dll")]
    public static extern bool BringWindowToTop(IntPtr hWnd);
    [DllImport("user32.dll")]
    public static extern IntPtr GetForegroundWindow();
    [DllImport("user32.dll")]
    public static extern bool GetWindowRect(IntPtr hWnd, out RECT lpRect);

    [StructLayout(LayoutKind.Sequential)]
    public struct RECT {
        public int Left, Top, Right, Bottom;
    }

    public static void CaptureWindow(IntPtr hwnd, string filename) {
        ShowWindow(hwnd, 3); // SW_MAXIMIZE
        System.Threading.Thread.Sleep(1000);
        SetForegroundWindow(hwnd);
        System.Threading.Thread.Sleep(500);

        RECT rect;
        GetWindowRect(hwnd, out rect);
        int width = rect.Right - rect.Left;
        int height = rect.Bottom - rect.Top;

        if (width <= 0 || height <= 0) {
            // fallback: full screen
            var screen = System.Windows.Forms.Screen.PrimaryScreen.Bounds;
            width = screen.Width;
            height = screen.Height;
            rect.Left = 0;
            rect.Top = 0;
        }

        var bmp = new Bitmap(width, height);
        using (var g = Graphics.FromImage(bmp)) {
            g.CopyFromScreen(rect.Left, rect.Top, 0, 0, new Size(width, height));
        }
        bmp.Save(filename, ImageFormat.Png);
    }
}
"@ -ReferencedAssemblies "System.Windows.Forms", "System.Drawing"

# Find msrdc (RViz2 in WSL GUI)
$proc = Get-Process msrdc -ErrorAction SilentlyContinue | Where-Object { $_.MainWindowHandle -ne 0 }
if ($proc) {
    Write-Output "Found msrdc (RViz2): handle=$($proc.MainWindowHandle)"
    [ScreenCapture]::CaptureWindow($proc.MainWindowHandle, "C:\Users\USER\Desktop\rviz_only.png")
    Write-Output "Captured to rviz_only.png"
} else {
    Write-Output "msrdc not found"
    Get-Process | Where-Object { $_.MainWindowHandle -ne 0 } | Select ProcessName, Id, MainWindowTitle
}
