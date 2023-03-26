namespace HoloArch.HoloScan
{
    using System;
    using System.Collections.Generic;
    using System.Runtime.InteropServices;

    public class SerialDevice : Base.Object
    {
        private static readonly Base.Deleter serialReleaser;
        
        public SerialDevice(IntPtr ptr)
            :base(ptr, serialReleaser)
        {
        }

        public static List<int> GetConnectedDevices(serial_config cfg)
        {
            throw new NotImplementedException();
        }

        public bool Start()
        {
            return NativeMethods.hs_start_serial_device(Handle);
        }

        public bool Start(serial_config cfg)
        {
            return NativeMethods.hs_start_serial_device_with_config(Handle, cfg);
        }

        public void Stop()
        {
            NativeMethods.hs_stop_serial_device(Handle);
        }

        public void SendCommand(string command)
        {
            NativeMethods.hs_serial_command(Handle, command);
        }
    }
}