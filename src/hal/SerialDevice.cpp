#ifndef SERIALDEVICE_HPP
#define SERIALDEVICE_HPP

#include "SerialDevice.hpp"

namespace hal
{
	SerialDevice::SerialDevice(const char* devname, char* mode, int bdrate) :
		mode(mode), bdrate(bdrate)
	{
		cport_n = get_port_number(devname);
	}

	SerialDevice::SerialDevice(int cport_n, int bdrate) :
		cport_n(cport_n), bdrate(bdrate)
	{

	}

	SerialDevice::SerialDevice(const char* devname, int bdrate) :
		bdrate(bdrate)
	{
		cport_n = get_port_number(devname);
	}

	SerialDevice::SerialDevice(int cport_n)
		:cport_n(cport_n)
	{

	}

	bool SerialDevice::start()
	{
		int success = RS232_OpenComport(cport_n, bdrate, mode, 0);

		if (!success)
		{
			success = 1;
		}
		else
			success = 0;

		return success;
	}

	bool SerialDevice::start(serial_config& cfg)
	{
		bdrate = cfg.bdrate;
		delete mode;
		mode = new char[] { cfg.data_bit, cfg.parity_bit, cfg.stop_bit, 0 };
		int success = RS232_OpenComport(cport_n, bdrate, mode, 0);

		if (!success)
		{
			success = 1;
		}
		else
			success = 0;

		return success;
	}

	void SerialDevice::start(common::CallBack& cb)
	{
		int success = RS232_OpenComport(cport_n, bdrate, mode, 0);

		if (success)
		{
			cb.on_success();
		}
		else
		{
			cb.on_error(-1, "unable to open comport number " + cport_n);
		}
	}

	void SerialDevice::cputs(const char* text)
	{
		RS232_cputs(cport_n, text);
	}

	int SerialDevice::get_port_number(const char* devname)
	{
		return RS232_GetPortnr(devname);
	}

	int SerialDevice::poll_port(unsigned char* buf, int size)
	{
		return RS232_PollComport(cport_n, buf, size);
	}

	void SerialDevice::stop()
	{
		RS232_CloseComport(cport_n);
	}
}

#endif // !SERIALDEVICE_HPP
