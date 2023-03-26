#ifndef SENSOR_HPP
#define SENSOR_HPP

#include "CallBack.hpp"
#include "../logic/ProcessingBlock.hpp"
#include "../logic/data-structures/conversion_proxy.hpp"
#include <thread>

namespace common
{
class Sensor
{
public:
    Sensor(std::vector<logic::logic_commons::PROCESSING_BLOCK> &logic_blocks){}
    Sensor(){}
    virtual void start(CallBack &cb) = 0;
    virtual void start(CallBack &cb, bool relocalisation) = 0;

    virtual void stop()
    {
        is_running = false;

        if (worker.joinable())
        {
            worker.join();
        }

        if (processing_worker.joinable())
        {
            processing_worker.join();
        }

        if (algorithm != nullptr)
        {
            delete algorithm;
        }
    }

    virtual bool is_streaming()
    {
        return is_running;
    }

    virtual logic::data_structures::conversion_proxy get_frame() = 0;
    virtual logic::data_structures::conversion_proxy get_frame(long time_stamp) = 0;

    void set_algorithm(logic::ProcessingBlock<float*, logic::data_structures::conversion_proxy> *algo)
    {
        algorithm = algo;
    }
protected:
    virtual void polling_action() = 0;
    virtual void processing_action() = 0;
    bool is_running;
    std::thread worker;
    std::thread processing_worker;
    CallBack *callback;
    logic::ProcessingBlock<float*, logic::data_structures::conversion_proxy> *algorithm = nullptr;
};
}

#endif // SENSOR_HPP
