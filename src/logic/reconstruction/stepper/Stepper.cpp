#include "Stepper.hpp"

namespace logic
{
	namespace reconstruction
	{
		namespace stepper
		{
			Stepper::Stepper(const char* devname, char* mode, int bdrate)
				: hal::SerialDevice(devname, mode, bdrate)
			{
			}

			Stepper::Stepper(int cport_n, int bdrate)
				: hal::SerialDevice(cport_n, bdrate)
			{
			}

			Stepper::Stepper(const char* devname, int bdrate)
				: hal::SerialDevice(devname, bdrate)
			{

			}

			Stepper::Stepper(int cport_n)
				: hal::SerialDevice(cport_n, bdrate)
			{

			}

			void Stepper::start(common::CallBack& cb)
			{
				hal::SerialDevice::start(cb);
			}

			bool Stepper::start()
			{
				return hal::SerialDevice::start();
			}

			bool Stepper::start(hal::serial_config& cfg)
			{
				return hal::SerialDevice::start(cfg);
			}

			std::pair<bool, Eigen::Matrix4d> Stepper::rotate_yaw(const int yaw)
			{
				std::string command = "";

				if (yaw != 0)
				{
					if (yaw < 0)
						command = "<L" + std::to_string((int)abs(yaw)) + ">";
					else
						command = "<R" + std::to_string((int)abs(yaw)) + ">";

					std::cout << "yawing " << yaw << " deg" << std::endl;
					hal::SerialDevice::cputs(command.c_str());
					std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
				}

				Eigen::Matrix4d rot = Eigen::Matrix4d::Identity();
				rot.topLeftCorner(3, 3) =
					utils::camera_transforms::
					euler_2_quaternion(0, -(double)yaw * deg_to_rad, 0).toRotationMatrix();
				return std::make_pair(true, rot);
			}

			std::pair<bool, Eigen::Matrix4d> Stepper::rotate_pitch(const int pitch)
			{
				std::string command = "";

				if (pitch != 0)
				{
					if (pitch < 0)
						command = "<D" + std::to_string((int)abs(pitch)) + ">";
					else
						command = "<U" + std::to_string((int)abs(pitch)) + ">";

					std::cout << "pitching " << pitch << " deg" << std::endl;
					hal::SerialDevice::cputs(command.c_str());
					std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
				}

				Eigen::Matrix4d rot = Eigen::Matrix4d::Identity();
				rot.topLeftCorner(3, 3) =
					utils::camera_transforms::
					euler_2_quaternion(-(double)pitch * deg_to_rad, 0, 0).toRotationMatrix();

				return std::make_pair(true, rot);
			}

			int Stepper::find_device()
			{
				unsigned char buf[4096];
				int res = -1;
				int n;
				long dur;

				class lg : public common::Loggable
				{
				public:
					void log(std::string s) { logger->log(s); }
				}local_logger;

				for (int i = 0; i < number_of_ports; i++)
				{
					auto begin = std::chrono::duration_cast<std::chrono::milliseconds>(
						std::chrono::time_point_cast<std::chrono::milliseconds>(
							std::chrono::high_resolution_clock::now()).time_since_epoch()).count();

					auto dev = SerialDevice(i, bdrate);

					if (dev.start())
					{
						dur = 0;

						while (dur < 50)
						{
							n = dev.poll_port(buf, 4095);

							if (n > 0)
							{
								buf[n] = 0;
								std::string line(buf, buf + n);

								if (line.find("Motors") != std::string::npos)
								{
									local_logger.log("Found HoloScanner on port number " + std::to_string(i) + ".");
									res = i;
									break;
								}
							}

							dur = std::chrono::duration_cast<std::chrono::milliseconds>(
								std::chrono::time_point_cast<std::chrono::milliseconds>(
									std::chrono::high_resolution_clock::now()).time_since_epoch()).count() - begin;
						}
					}

					dev.stop();

					if (res > 0)
					{
						break; // for loop
					}
				}

				return res;
			}

			std::vector<int> Stepper::get_connected_devices(hal::serial_config& cfg)
			{
				unsigned char buf[4096];
				std::vector<int> res;
				int n;
				long dur;

				class lg : public common::Loggable
				{
				public:
					void log(std::string s) { logger->log(s); }
				}local_logger;

				for (int i = 0; i < number_of_ports; i++)
				{
					auto begin = std::chrono::duration_cast<std::chrono::milliseconds>(
						std::chrono::time_point_cast<std::chrono::milliseconds>(
							std::chrono::high_resolution_clock::now()).time_since_epoch()).count();

					auto dev = SerialDevice(i);

					if (dev.start(cfg))
					{
						dur = 0;

						while (dur < 50)
						{
							n = dev.poll_port(buf, 4095);

							if (n > 0)
							{
								buf[n] = 0;
								std::string line(buf, buf + n);

								if (line.find("Motors") != std::string::npos)
								{
									local_logger.log("Found HoloScanner on port number " + std::to_string(i) + ".");
									res.push_back(i);
									break;
								}
							}

							dur = std::chrono::duration_cast<std::chrono::milliseconds>(
								std::chrono::time_point_cast<std::chrono::milliseconds>(
									std::chrono::high_resolution_clock::now()).time_since_epoch()).count() - begin;
						}
					}

					dev.stop();
				}

				return res;
			}

			void Stepper::send_command(const char* command)
			{
				hal::SerialDevice::cputs(command);
				std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
			}

			void Stepper::stop()
			{
				hal::SerialDevice::stop();
			}
		} // namespace encoder
	} // namespace reconstruction
} // naemspace logic