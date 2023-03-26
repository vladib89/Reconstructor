namespace HoloArch.HoloScan
{
    using System;
    using System.Collections;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Runtime.InteropServices;

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    public class depth_stream_config : stream_config
    {
		public enum DepthFormat
		{
			Z16
		};

		public int depth_fps;
		public int depth_height;
		public int depth_width;
		public DepthFormat depth_format;
		public ColorFormat color_format;
		public bool enable_gyro = false;
		public bool enable_accelerometer = false;
	}
}