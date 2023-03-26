namespace HoloArch.HoloScan
{
    using System;
    using System.Collections;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Runtime.InteropServices;

    public class Stepper : SerialDevice
    {
        public Stepper(int cportNum)
            : base(NativeMethods.hs_create_stepper(cportNum))
        {
        }

        new public static List<int> GetConnectedDevices(serial_config cfg)
        {
            IntPtr list = (IntPtr)0;
            uint length = 5;
            NativeMethods.hs_get_available_steppers(out list, out length, cfg);
            int[] data = new int[length];
            Marshal.Copy(list, data, 0, (int)length);
            Marshal.FreeCoTaskMem(list);

            return new List<int>(data);
        }
    }
}