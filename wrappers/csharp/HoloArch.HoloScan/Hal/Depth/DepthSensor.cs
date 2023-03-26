namespace HoloArch.HoloScan
{
    using System;
    using System.Collections;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Runtime.InteropServices;

    public abstract class DepthSensor : Sensor
    {
        public DepthSensor()
            : base(new IntPtr())
        {

        }

        public void Start(CallBack cb, depth_stream_config cfg)
        {
            throw new NotImplementedException();
        }

        public void Stop(CallBack cb)
        {
            throw new NotImplementedException();
        }

        public static List<string> get_availabe_depth_devices()
        {
            IntPtr listPtr = NativeMethods.hs_get_available_depth_devices();
            string listString = System.Runtime.InteropServices.Marshal.PtrToStringAnsi(listPtr);
            List<string> res = new List<string>();

            if (listString != null)
            {
               res = new List<string>(listString.Split(
                            new[] { "\r\n", "\r", "\n" },
                            StringSplitOptions.None));
            }

            return res;
        }
    }
}