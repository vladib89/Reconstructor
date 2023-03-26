#ifndef SERVER_HPP
#define SERVER_HPP

#include "data-structures/task.hpp"
#include "../common/FixedQueue.hpp"
#include <unordered_map>
#include <unordered_set>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <future>

namespace logic
{
	class server
	{
	public:
		bool add_task(data_structures::task* t, bool is_lock_acquired = false);
		void *doSomething;
		void wait_all_complete();
		static server* get_instance();
	private:
		server();
		const unsigned int processor_count = std::thread::hardware_concurrency();
		static server* instance;
		std::mutex pending_mutex;
		std::condition_variable pending_condition;
		std::unordered_set<unsigned> pending;
		common::FixedQueue<data_structures::task*, 100> waiting;
		unsigned next_id = 0;
		void notify_complete(unsigned id);
	};
}

#endif // !SERVER_HPP
