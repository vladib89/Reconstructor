namespace HoloArch.HoloScan
{
    using System;
    using System.Runtime.InteropServices;

	public class Frame
	{
		protected IntPtr handle;
		protected int size;
		
		public Frame(IntPtr handle, int size, int height, int width)
        {
			this.handle = handle;
			this.size = size;
			Width = width;
			Height = height;
        }

		public int Width { get; private set; }
		
		public int Height { get; private set; }

		IntPtr getData()
        {
			return handle;
        }

		void free()
        {

        }
	}
}