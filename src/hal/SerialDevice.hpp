#ifndef SERIAL_DEVICE_HPP
#define SERIAL_DEVICE_HPP

#include "rs232/rs232.h"
#include "serial_config.hpp"
#include "../common/CallBack.hpp"
#include "../common/Loggable.hpp"

namespace hal
{
	class SerialDevice : public common::Loggable
	{
	private:
		int cport_n = 0;
		int bdrate = 115200;
		char* mode = new char[] { '8', 'N', '1', 0 };
	public:
		SerialDevice(
			const char* devname, 
			char* mode, 
			int bdrate);
		SerialDevice(
			const char* devname, 
			int bdrate);
		SerialDevice(
			int cport_n, 
			int bdrate);
		SerialDevice(
			int cport_n);
		bool start();
		bool start(serial_config& cfg);
		void stop();
		void start(common::CallBack& cb);
		void cputs(const char* text);
		int poll_port(
			unsigned char* buf, 
			int size);
	protected:
		int get_port_number(
			const char* devname);
	};
}

#endif // !SERIALDEVICE_HPP
