#ifndef MENU_HPP
#define MENU_HPP

#include <stdlib.h>

namespace common
{
	void clear_console();
	typedef void (*Menu_Processing_Function_Pointer)(void);
	
	struct Menu_Option
	{
		char choice;
		char const* p_selection_text;
		Menu_Processing_Function_Pointer p_processing_function;
	};
}

#endif // !MENU_HPP
