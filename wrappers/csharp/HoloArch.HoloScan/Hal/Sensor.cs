namespace HoloArch.HoloScan
{
    using System;
    using System.Collections;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Runtime.InteropServices;

    public abstract class Sensor : Base.Object
    {
        protected CallBack callBack;
        protected string name = string.Empty;
        protected string serial = string.Empty;
        protected static readonly Base.Deleter sensorReleaser;

        public Sensor(IntPtr ptr)
            : base(ptr, sensorReleaser)
        {
        }
    }
}