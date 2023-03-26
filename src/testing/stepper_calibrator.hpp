#ifndef STEPPER_CALIBRATOR_HPP
#define STEPPER_CALIBRATOR_HPP

#include "../logic/reconstruction/reconstruction_utils/image_transforms.hpp"
#include "../logic/reconstruction/config.hpp"
#include "../logic/reconstruction/reconstruction_utils/miscellaneous.hpp"
#include "../logic/reconstruction/stepper/stepper_program.hpp"
#include "../depth/depth_sensor.hpp"
#include "../depth/depth_stream_config.hpp"
#include "../depth/realsense/RsDepthDevice.hpp"
#include "../common/Loggable.hpp"
#include "Eigen/Dense"
#include "open3d/Open3D.h"

namespace stepper_calibrator
{
	void func1();
	void func2();
	typedef void (*callback_function)(void);
}

#endif // !STEPPER_CALIBRATOR_HPP
