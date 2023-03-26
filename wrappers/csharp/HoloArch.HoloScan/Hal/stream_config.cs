namespace HoloArch.HoloScan
{
    using System;
    using System.Collections;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Runtime.InteropServices;

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public class stream_config
    {
        public enum ColorFormat
        {
            RGB8,
            BGR8
        };

        [MarshalAs(UnmanagedType.LPStr, SizeConst = 50)]
        public string device_name;
        [MarshalAs(UnmanagedType.LPStr, SizeConst = 50)]
        public string serial_number;
        public int color_height;
        public int color_width;
        public int color_fps;
    }
}