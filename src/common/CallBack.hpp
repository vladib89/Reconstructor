#ifndef CALLBACK_H
#define CALLBACK_H

#include <iostream>

namespace common
{
	class CallBack
	{
	public:
		virtual void on_success() = 0;
		virtual void on_error(int result, std::string reason) = 0;
	};
} // namespace common

#endif // CALLBACK_H
