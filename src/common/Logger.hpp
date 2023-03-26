#ifndef LOGGER_HPP
#define LOGGER_HPP

#include "global.hpp"
#include  "../logic/LogicCommons.hpp"
#include <string>
#include <iostream>
#include <fstream>
#include <mutex>

namespace common
{
class Logger
{
private:
	Logger();
	std::mutex mtx;
	static Logger* instance;
	std::string getCurrentDateTime(std::string s);
    const std::string log_path = "../logs/log.txt";
    unsigned int current_number_of_lines = 0;
    const unsigned int max_number_of_lines = 400;
public:
	static Logger *get_instance()
	{
		static Logger s;
		return &s;
	}

	void log(std::string logMsg);
	Logger(const Logger&) = delete;
	void operator =(const Logger&) = delete;
};
}

#endif // LOGGER_HPP
