#include "tests.h"
#include "matplotlibcpp.h"
using namespace matplotlibcpp;

int main()
{
	init_robot_arm(3, 3);
	set_base_position(0, 2);
	set_end_position(0, 2, 10, 2);

    return 0;
}