namespace HoloArch.HoloScan.Base
{
    using System;
    using System.Diagnostics;

    public abstract class Object : IDisposable
    {
        [DebuggerBrowsable(DebuggerBrowsableState.Never)]
        internal readonly DeleterHandle m_instance;

        protected Object(IntPtr ptr, Deleter deleter)
        {
            if (ptr == IntPtr.Zero)
            {
                throw new ArgumentNullException(nameof(ptr));
            }

            m_instance = new DeleterHandle(ptr, deleter);
        }

        public IntPtr Handle
        {
            get
            {
                if (m_instance.IsInvalid)
                {
                    throw new ObjectDisposedException(GetType().Name);
                }

                return m_instance.Handle;
            }
        }

        public void Dispose()
        {
            this.Dispose(true);
        }

        protected virtual void Dispose(bool disposing)
        {
            m_instance.Dispose();
        }

        internal void Reset(IntPtr ptr, Deleter deleter)
        {
            m_instance.Reset(ptr, deleter);
        }
    }
}