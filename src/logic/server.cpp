#include "server.hpp"

namespace logic
{
	server* server::instance = 0;

	server* server::get_instance()
	{
		if (instance == 0)
		{
			instance = new server();
		}

		return instance;
	}

	server::server()
	{
	}

	void server::wait_all_complete()
	{
		auto none_left = [&] {
			return pending.empty();
		};

		auto lock = std::unique_lock<std::mutex>(pending_mutex);
		pending_condition.wait(lock, none_left);
	}

	void server::notify_complete(unsigned id)
	{
		auto lock = std::unique_lock<std::mutex>(pending_mutex);
		pending.erase(id);

		if (!waiting.empty())
			add_task(waiting.dequeue(), true);

		if (pending.empty())
		{
			pending_condition.notify_all();
		}
	}

	bool server::add_task(data_structures::task* t, bool is_lock_aquired)
	{
		bool res = true;

		if (pending.size() == processor_count)
		{
			res = false;
			waiting.enqueue(t);
		}
		else
		{
			std::unique_lock<std::mutex> lock;

			if (!is_lock_aquired)
				lock = std::unique_lock<std::mutex>(pending_mutex);

			auto id = next_id++;
			auto f = std::thread([this, t, id] {
				t->exectue_task();
				delete t;
				this->notify_complete(id);
				});
			f.detach();
			pending.emplace(id);
		}

		return res;
	}
}