#include "testing/reconstructor_tester.hpp"
#include "testing/stepper_calibrator.hpp"

int main(int argc, char* argv[])
{

	logic::reconstruction::testing::reconstructor_tester tester;
	tester.start_tester(logic::reconstruction::testing::mode::reg);
	stepper_calibrator::func1();
	//void** hitems;
	//size_t size;
}
