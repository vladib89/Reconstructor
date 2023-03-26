#ifndef RSDEVICE_HPP
#define RSDEVICE_HPP

#include "Sensor.hpp"
#include "../logic/data-structures/frame.hpp"
#include <librealsense2/rs.hpp>

namespace common
{
class RsDevice
{
public:
    RsDevice(){}
    RsDevice(std::vector<logic::logic_commons::PROCESSING_BLOCK> *logic_blocks){}
    bool get_frames(std::pair <logic::data_structures::frame, logic::data_structures::frame>& res) {}
    rs2::pipeline pipe;
protected:
    rs2::config cfg;
};
}

#endif // RSDEVICE_HPP

