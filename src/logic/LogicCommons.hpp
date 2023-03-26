#ifndef LOGICCOMMONS_HPP
#define LOGICCOMMONS_HPP
#define NOMINMAX

#include "../common/global.hpp"
#include <sys/stat.h>
#include <string>
#include <regex>
#include <sstream>
#include <iostream>
#ifdef _WIN32
#include <direct.h>
#include <conio.h>
#include "../../include/dirent.h"
#elif __unix__ 
#include <dirent.h>
#endif

namespace logic
{
	namespace logic_commons
	{
		enum SENSOR_TYPE
		{
			DEFAULT_SENSOR,
			T265,
			HOLOARCH_TRACKING,
			D435
		};

		enum PROCESSING_BLOCK
		{
			DEFAULT_BLOCK,
			KALMAN,
			BOUNDING_POLYHEDRON,
			TRILATERATION,
			TDOA_LOCATION
		};

		template<typename T>
		void delete_2d_arr(T** arr, size_t m);
		int _mkdir(const char* path);
		bool file_exists(const std::string& name);
		int get_files_list_with_extension(const std::string& path, const std::string& ext, std::vector<std::string>& list);
		void set_params(int argc, char* argv[]);
		float* check_if_valid_return_matches(std::regex& r, std::smatch& m,
			std::string& input, size_t& number_of_matches);
		int mkdir(const char* path);
		void recursive_delete(const char* dir);
		bool is_not_digit(char c);
		bool numeric_string_compare(const std::string& s1, const std::string& s2);
	}
}

#endif // LOGICCOMMONS_HPP
