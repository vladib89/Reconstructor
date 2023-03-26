#include "LogicCommons.hpp"

namespace logic
{
	namespace logic_commons
	{
		int _mkdir(const char* path)
		{
#ifdef _WIN32
			return ::_mkdir(path);
#else
#if _POSIX_C_SOURCE
			return ::mkdir(path);
#else
			return ::mkdir(path, 0755);
#endif
#endif
		}

		int get_files_list_with_extension(const std::string& path, const std::string& ext, std::vector<std::string>& list)
		{
			DIR* dir;
			struct dirent* entry;

			if ((dir = opendir(path.c_str())) != NULL) {
				/* print all the files and directories within directory */
				while ((entry = readdir(dir)) != NULL) {
					if (entry->d_type == DT_REG) {		// if entry is a regular file
						std::string fname = entry->d_name;	// filename
															// if filename's last characters are extension
						if (fname.find(ext, (fname.length() - ext.length())) != std::string::npos)
							list.push_back(path + fname);		// add filename to results vector
					}
				}
				closedir(dir);
			}
			else {
				/* could not open directory */
				perror("");
				return EXIT_FAILURE;
			}

			return 1;
		}

#ifdef  _WIN32
		void recursive_delete(const char* dir)
		{
			std::string dir_path(dir);
			std::string command = "rd /s /q \"" + dir_path + "\"";
			system(command.c_str());
		}
#elif __unix__ 
		int recursive_delete(const char* dir)
		{
			int ret = 0;
			FTS* ftsp = NULL;
			FTSENT* curr;

			// Cast needed (in C) because fts_open() takes a "char * const *", instead
			// of a "const char * const *", which is only allowed in C++. fts_open()
			// does not modify the argument.
			char* files[] = { (char*)dir, NULL };

			// FTS_NOCHDIR  - Avoid changing cwd, which could cause unexpected behavior
			//                in multithreaded programs
			// FTS_PHYSICAL - Don't follow symlinks. Prevents deletion of files outside
			//                of the specified directory
			// FTS_XDEV     - Don't cross filesystem boundaries
			ftsp = fts_open(files, FTS_NOCHDIR | FTS_PHYSICAL | FTS_XDEV, NULL);
			if (!ftsp) {
				fprintf(stderr, "%s: fts_open failed: %s\n", dir, strerror(curr->fts_errno));
				ret = -1;
				goto finish;
			}

			while ((curr = fts_read(ftsp))) {
				switch (curr->fts_info) {
				case FTS_NS:
				case FTS_DNR:
				case FTS_ERR:
					fprintf(stderr, "%s: fts_read error: %s\n",
						curr->fts_accpath, strerror(curr->fts_errno));
					break;

				case FTS_DC:
				case FTS_DOT:
				case FTS_NSOK:
					// Not reached unless FTS_LOGICAL, FTS_SEEDOT, or FTS_NOSTAT were
					// passed to fts_open()
					break;

				case FTS_D:
					// Do nothing. Need depth-first search, so directories are deleted
					// in FTS_DP
					break;

				case FTS_DP:
				case FTS_F:
				case FTS_SL:
				case FTS_SLNONE:
				case FTS_DEFAULT:
					if (remove(curr->fts_accpath) < 0) {
						fprintf(stderr, "%s: Failed to remove: %s\n",
							curr->fts_path, strerror(curr->fts_errno));
						ret = -1;
					}
					break;
				}
			}

		finish:
			if (ftsp) {
				fts_close(ftsp);
			}
	}
#endif //  _WIN32

		float* check_if_valid_return_matches(std::regex& r, std::smatch& m,
			std::string& input, size_t& number_of_matches)
		{
			float* res = nullptr;
			auto m_begin = std::sregex_iterator(input.begin(), input.end(), r);
			auto m_end = std::sregex_iterator();
			number_of_matches = std::distance(m_begin, m_end);
			size_t idx = 0;

			if (number_of_matches > 0)
			{
				res = new float[number_of_matches];

				for (std::sregex_iterator i = m_begin; i != m_end; i++)
				{
					std::smatch m = *i;
					res[idx] = std::stof(m.str());
					idx++;
				}
			}

			return res;
		}

		void set_params(int argc, char* argv[])
		{
			if (argc > 1)
			{
				std::string arg;

				for (size_t i = 1; i < argc; i++)
				{
					arg = argv[i];

					if (arg == "log")
					{
						is_log = true;
					}
					else if (arg == "process")
					{
						is_proc = true;
					}
				}
			}
		}

		template<typename T>
		void delete_2d_arr(T** arr, size_t m)
		{
			for (size_t i = 0; i < m; i++)
			{
				delete[] arr[i];
			}

			delete[] arr;
		}

		bool file_exists(const std::string& name)
		{
			struct stat buffer;
			return (stat(name.c_str(), &buffer) == 0);
		}

		int mkdir(const char* path)
		{
			if (file_exists(path))
				recursive_delete(path);

			return _mkdir(path);
		}

		bool is_not_digit(char c)
		{
			return !std::isdigit(c);
		}

		bool numeric_string_compare(const std::string& s1, const std::string& s2)
		{
			// handle empty strings...

			std::string::const_iterator it1 = s1.begin(), it2 = s2.begin();

			if (std::isdigit(s1[0]) && std::isdigit(s2[0])) {
				int n1, n2;
				std::stringstream ss(s1);
				ss >> n1;
				ss.clear();
				ss.str(s2);
				ss >> n2;

				if (n1 != n2) return n1 < n2;

				it1 = std::find_if(s1.begin(), s1.end(), is_not_digit);
				it2 = std::find_if(s2.begin(), s2.end(), is_not_digit);
			}

			return std::lexicographical_compare(it1, s1.end(), it2, s2.end());
		}
}
}
