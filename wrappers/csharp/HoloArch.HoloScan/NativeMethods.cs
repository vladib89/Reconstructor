namespace HoloArch.HoloScan
{
    using System;
    using System.Collections.Generic;
    using System.Reflection.Emit;
    using System.Runtime.InteropServices;
    using System.Security;
    using System.Security.Permissions;

    internal static class NativeMethods
    {
#if DEBUG
        private const string dllName = "holoscan";
#else
        private const string dllName = "holoscan";
#endif

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate IntPtr MemCpyDelegate(IntPtr dest, IntPtr src, int count);

        internal static readonly MemCpyDelegate Memcpy = MemCpy.GetMethod();

        [System.Security.SuppressUnmanagedCodeSecurity]
        internal static class MemCpy
        {
            internal static MemCpyDelegate GetMethod()
            {
                switch (System.Environment.OSVersion.Platform)
                {
                    case PlatformID.Win32NT:
                        return win_memcpy;
                    case PlatformID.Unix:
                    case PlatformID.MacOSX:
                        return unix_memcpy;
                    default:
                        throw new PlatformNotSupportedException(System.Environment.OSVersion.ToString());
                }
            }
        }

        [DllImport("libc", EntryPoint = "memcpy", CallingConvention = CallingConvention.Cdecl, SetLastError = false)]
        internal static extern IntPtr unix_memcpy(IntPtr dest, IntPtr src, int count);

        [DllImport("msvcrt.dll", EntryPoint = "memcpy", CallingConvention = CallingConvention.Cdecl, SetLastError = false)]
        internal static extern IntPtr win_memcpy(IntPtr dest, IntPtr src, int count);

        #region serial comms
        
        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern bool hs_start_serial_device(IntPtr stepper);

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern bool hs_start_serial_device_with_config(IntPtr stepper, serial_config cfg);

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void hs_serial_command(IntPtr serialDevice, string command);

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void hs_stop_serial_device(IntPtr serialDevice);

        #region stepper
        // TODO: make generic, search by handshake
        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void hs_get_available_steppers(out IntPtr pData, out uint length, serial_config cfg);

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr hs_create_stepper(int cportNum);

        // TODO: implement for sytntactic testing
        /*[DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr hs_stepper_command(IntPtr stepper, string command);*/

        #endregion // stepper

        #endregion // serial comms

        #region depth

        [DllImport(dllName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr hs_get_available_depth_devices();

        #endregion // depth
    }
}