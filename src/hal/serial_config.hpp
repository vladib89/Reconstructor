#ifndef SERIAL_CONFIG_HPP
#define SERIAL_CONFIG_HPP

namespace hal
{
	struct serial_config
	{
		int bdrate = 115200;
		char data_bit = '8';
		char parity_bit = 'N';
		char stop_bit = '1';
	};
}

#endif // !SERIAL_CONFIG_HPP
