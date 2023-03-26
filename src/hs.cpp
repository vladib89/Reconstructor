#include "api.hpp"
#include "hal/SerialDevice.hpp"
#include "hal/serial_config.hpp"
#include "depth/depth_sensor.hpp"
#include "logic/reconstruction/stepper/Stepper.hpp"

#include <sstream>

#pragma region serial

EXPORT bool hs_start_serial_device(hal::SerialDevice* serial_device)
{
	return serial_device->start();
}

EXPORT bool hs_start_serial_device_with_config(
	hal::SerialDevice* serial_device,
	hal::serial_config cfg)
{
	return serial_device->start(cfg);
}

EXPORT void hs_serial_command(hal::SerialDevice* serial_device, const char* command)
{
	serial_device->cputs(command);
}

EXPORT void hs_stop_serial_device(hal::SerialDevice* serial_device)
{
	serial_device->stop();
}
#pragma endregion serial

#pragma region stepper

EXPORT void hs_get_available_steppers(
	int** hItems, size_t* size, hal::serial_config cfg)
{
	auto items =
		std::vector<int>(logic::reconstruction::stepper::Stepper::get_connected_devices(cfg));
	*size = items.size();
	auto len = (*size) * sizeof(int);
	*hItems = static_cast<int*>(malloc(len));
	memcpy(*hItems, items.data(), len);
}

EXPORT logic::reconstruction::stepper::Stepper* hs_create_stepper(int cport_n)
{
	return new logic::reconstruction::stepper::Stepper(cport_n);
}
#pragma endregion stepper

#pragma region Depth
EXPORT const char* hs_get_available_depth_devices()
{
	auto dev_list = depth::depth_sensor::get_available_depth_sensors();
	auto it = dev_list.begin();
	std::stringstream res;

	while (it != dev_list.end())
	{
		if (it != dev_list.begin())
			res << std::endl;

		res << it->first << std::endl;
		res << it->second;
		it++;
	}

	return res.str().c_str();
}
#pragma endregion Detph

//int main(int argc, char* argv[])
//{

//	//logic::reconstruction::testing::reconstructor_tester tester;
//	//tester.start_tester(logic::reconstruction::testing::mode::REG);
//	//stepper_calibrator::func1();
//	void** hitems;
//	size_t size;
//	std::string arch = hs_get_available_depth_devices();
//	std::cout << arch << std::endl;
//}
