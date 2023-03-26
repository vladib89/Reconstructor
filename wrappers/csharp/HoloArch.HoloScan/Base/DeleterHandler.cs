namespace HoloArch.HoloScan.Base
{
    using System;
    using System.Diagnostics;
    using System.Runtime.InteropServices;
    using System.Security;

    [SuppressUnmanagedCodeSecurity]
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate void Deleter(IntPtr ptr);

    internal sealed class DeleterHandle : IDisposable
    {
        private IntPtr handle;
        private Deleter deleter;

        public IntPtr Handle => handle;
        
        public bool IsInvalid => handle == IntPtr.Zero;

        public DeleterHandle(IntPtr ptr, Deleter deleter)
        {
            handle = ptr;
            this.deleter = deleter;
        }

        public void SetHandleAsInvalid()
        {
            handle = IntPtr.Zero;
            GC.SuppressFinalize(this);
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        public void Dispose(bool disposing)
        {
            if (handle == IntPtr.Zero)
            {
                return;
            }

            deleter?.Invoke(handle);
            handle = IntPtr.Zero;
        }

        internal void Reset(IntPtr ptr)
        {
            handle = ptr;
            GC.ReRegisterForFinalize(this);
        }

        internal void Reset(IntPtr ptr, Deleter deleter)
        {
            this.handle = ptr;
            this.deleter = deleter;
            //GC.ReRegisterForFinalize(this);
        }

        ~DeleterHandle()
        {
            //Console.WriteLine($"~{handle} {deleter?.Method}");
            Dispose(false);
        }
    }
}