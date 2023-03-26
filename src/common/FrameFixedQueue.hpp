#ifndef FRAMEFIXEDQUEUE_HPP
#define FRAMEFIXEDQUEUE_HPP

#include "FixedQueue.hpp"
#include "../logic/data-structures/frame.hpp"
#include "../logic/data-structures/motion_frame.hpp"

namespace common
{
	template<int MaxLen>
	class FrameFixedQueue :FixedQueue<std::tuple<logic::data_structures::frame,
		logic::data_structures::frame, logic::data_structures::motion_frame>, MaxLen>
	{
	public:
		void enqueue(const std::tuple<logic::data_structures::frame,
			logic::data_structures::frame, logic::data_structures::motion_frame>& value)
		{
			this->m.lock();

			if (this->size() == MaxLen)
			{
				std::tuple<logic::data_structures::frame, logic::data_structures::frame,
					logic::data_structures::motion_frame> elem = this->front();
				this->pop_front();

				std::get<0>(elem).free();
				std::get<1>(elem).free();
				std::get<2>(elem).free();
			}

			this->push_back(value);

			this->m.unlock();
		}

		using common::FixedQueue<std::tuple<logic::data_structures::frame,
			logic::data_structures::frame, logic::data_structures::motion_frame>, MaxLen>::dequeue;
		using common::FixedQueue<std::tuple<logic::data_structures::frame,
			logic::data_structures::frame, logic::data_structures::motion_frame>, MaxLen>::empty;
	};
}

#endif // !FRAMEFIXEDQUEUE_HPP
