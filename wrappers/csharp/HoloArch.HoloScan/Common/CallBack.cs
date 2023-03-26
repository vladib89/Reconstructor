namespace HoloArch.HoloScan
{
	using System.Runtime.InteropServices;

	[StructLayout(LayoutKind.Sequential)]
	public abstract class CallBack
    {
		public abstract void OnSuccess();
		public abstract void OnError(int result, string reason);
    }
}