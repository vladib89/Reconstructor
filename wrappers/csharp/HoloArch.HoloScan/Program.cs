using System;
using System.Collections.Generic;

namespace HoloArch.HoloScan
{
    class Program
    {
        public static void Main(string[] args)
        {
            // serial_config cfg = new serial_config();
            // cfg.bdrate = 9600;
            // cfg.data_bit = '8';
            // cfg.parity_bit = 'N';
            // cfg.stop_bit = '1';
            // var arch = Stepper.GetConnectedDevices(cfg);
            // var stepper = new Stepper(arch[0]);
            //var success = stepper.Start(cfg);
            // stepper.SendCommand("<R40>");
            // Console.ReadKey();
            var arch = DepthSensor.get_availabe_depth_devices();
        }
    }
}