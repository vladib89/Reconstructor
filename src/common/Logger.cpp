#include "Logger.hpp"

namespace common
{

Logger::Logger()
{
	if (logic::logic_commons::file_exists(log_path))
	{
		FILE *infile = fopen(log_path.c_str(), "r");
		int ch;

		while (EOF != (ch=getc(infile)))
			if ('\n' == ch)
				++current_number_of_lines;
	}
}

void Logger::log(std::string logMsg)
{
	mtx.lock();

	if (is_log)
	{
		std::string now = getCurrentDateTime("now");
		std::ofstream ofs;

		if (++current_number_of_lines > max_number_of_lines)
		{
            current_number_of_lines = 1;
			ofs.open(log_path.c_str(), std::ios_base::out);
		}
		else
		{
			ofs.open(log_path.c_str(), std::ios_base::out | std::ios_base::app);
		}

		ofs << now << '\t' << logMsg << '\n';
		ofs.close();
	}
	else
	{
		std::cout << logMsg << std::endl;
	}
	
	mtx.unlock();
}

std::string Logger::getCurrentDateTime(std::string s)
{
	time_t now = time(0);
	struct tm  tstruct;
	char  buf[80];
	tstruct = *localtime(&now);
	if(s=="now")
		strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);
	else if(s=="date")
		strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);
	return std::string(buf);
};
}
