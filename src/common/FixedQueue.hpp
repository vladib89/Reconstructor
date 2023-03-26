#ifndef FIXEDQUEUE_H
#define FIXEDQUEUE_H

#include <deque>
#include <iostream>
#include <mutex>
#include <queue>

namespace common
{
template <typename T, int MaxLen>
class FixedQueue : public std::deque<T>
{
public:
    void enqueue(const T& value)
    {
        m.lock();

        if (this->size() == MaxLen)
        {
            this->pop_front();
        }

        std::deque<T>::push_back(value);

        m.unlock();
    }

    T dequeue()
    {
        m.lock();
        T t = std::deque<T>::front();
        std::deque<T>::pop_front();
        m.unlock();
        return t;
    }

    bool empty()
    {
        bool res;
        m.lock();
        res = std::deque<T>::empty();
        m.unlock();

        return res;
    }

protected:
    std::mutex m;
};
}

#endif // FIXEDQUEUE_H
