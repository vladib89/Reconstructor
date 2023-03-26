#ifndef STEPPER_HPP
#define STEPPER_HPP

#include "stepper_modes.hpp"
#include "../../../hal/SerialDevice.hpp"
#include "../../../hal/serial_config.hpp"
#include "../reconstruction_utils/image_transforms.hpp"

#include <chrono>

namespace logic
{
	namespace reconstruction
	{
		namespace stepper
		{
			class Stepper : public hal::SerialDevice
			{
			private:
				// TODO: take care of unix
				static const int bdrate = 9600;
				static const int number_of_ports = 32;
				static const int wait_time = 3000;
				Eigen::Matrix4d camera_position = Eigen::Matrix4d::Identity();
			public:
				Stepper(const char* devname, char* mode, int bdrate);
				Stepper(const char* devname, int bdrate);
				Stepper(int cport_n, int bdrate);
				Stepper(int cport_n);
				static int find_device();
				static std::vector<int> get_connected_devices(hal::serial_config& cfg);
				void start(common::CallBack& cb);
				bool start(hal::serial_config& cfg);
				bool start();
				void stop();
				std::pair<bool, Eigen::Matrix4d> rotate_yaw(const int yaw);
				std::pair<bool, Eigen::Matrix4d> rotate_pitch(const int pitch);
				void send_command(const char* command);
			};
		} // namespace encoder
	} // namespace reconstruction
} // namespace logic

#endif // !STEPPER_HPP
