namespace HoloArch.HoloScan
{
    using System;
    using System.Collections;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Runtime.InteropServices;

    [StructLayout(LayoutKind.Sequential)]
    public struct serial_config
    {
        public int bdrate;
        public char data_bit;
        public char parity_bit;
        public char stop_bit;
    }
}