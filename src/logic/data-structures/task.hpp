#ifndef TASK_HPP
#define TASK_HPP

namespace logic
{
	namespace data_structures
	{
		struct task
		{
		public:
			virtual void exectue_task() = 0;
		};
	}
}

#endif // !TASK_HPP
