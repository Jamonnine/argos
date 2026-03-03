Add-Type @"
using System;
using System.Text;
using System.Drawing;
using System.Drawing.Imaging;
using System.Collections.Generic;
using System.Runtime.InteropServices;
public class WinCapture {
    public delegate bool EnumWindowsProc(IntPtr hWnd, IntPtr lParam);
    [DllImport("user32.dll")] public static extern bool EnumWindows(EnumWindowsProc f, IntPtr l);
    [DllImport("user32.dll")] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
    [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
    [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int n);
    [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
    [StructLayout(LayoutKind.Sequential)]
    public struct RECT { public int Left, Top, Right, Bottom; }

    public static void CaptureByTitle(string titlePart, string file) {
        IntPtr found = IntPtr.Zero;
        EnumWindows((hwnd, lp) => {
            if (!IsWindowVisible(hwnd)) return true;
            var sb = new StringBuilder(256);
            GetWindowText(hwnd, sb, 256);
            if (sb.ToString().Contains(titlePart)) { found = hwnd; return false; }
            return true;
        }, IntPtr.Zero);

        if (found == IntPtr.Zero) { Console.WriteLine("Not found: " + titlePart); return; }

        ShowWindow(found, 9);
        System.Threading.Thread.Sleep(600);
        SetForegroundWindow(found);
        System.Threading.Thread.Sleep(400);

        RECT rect;
        GetWindowRect(found, out rect);
        int w = rect.Right - rect.Left;
        int h = rect.Bottom - rect.Top;
        if (w <= 0) { w = 1400; h = 900; rect.Left = 0; rect.Top = 0; }

        var bmp = new Bitmap(w, h);
        using (var g = Graphics.FromImage(bmp)) {
            g.CopyFromScreen(rect.Left, rect.Top, 0, 0, new Size(w, h));
        }
        bmp.Save(file, ImageFormat.Png);
        Console.WriteLine("Saved: " + file + " (" + w + "x" + h + ")");
    }
}
"@ -ReferencedAssemblies "System.Windows.Forms","System.Drawing"

[WinCapture]::CaptureByTitle("RViz", "C:\Users\USER\Desktop\rviz_lidar.png")
