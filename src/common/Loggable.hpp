#ifndef LOGGABLE_HPP
#define LOGGABLE_HPP

#include "Logger.hpp"

namespace common
{
	class Loggable
	{
	protected:
		Logger* logger = Logger::get_instance();
	};
}

#endif // LOGGABLE_HPP
