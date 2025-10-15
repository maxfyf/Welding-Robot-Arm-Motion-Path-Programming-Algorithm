#include <iostream>
#include <vector>
#include "global.h"
#include "Beam.h"
using namespace std;

void print_beam_distance_cost(Output& beam_output)
{
	cout << "Beam Distance Cost: " << beam_output.distance_cost << endl << endl;
}

void print_beam_base_path()
{
	cout << "Beam Base Path:" << endl;
	for (auto& p : beam_base_path)
		cout << "(" << ((int)(p.x * 100)) / 100.0 << ", " << ((int)(p.y * 100)) / 100.0 << ")" << endl;
	cout << endl;
}

void print_beam_joint_path()
{
	cout << "Beam Joint Path:" << endl;
	for (auto& p : beam_joint_path)
		cout << "(" << ((int)(p.x * 100)) / 100.0 << ", " << ((int)(p.y * 100)) / 100.0 << ", " << ((int)(p.z * 100)) / 100.0 << ")" << endl;
	cout << endl;
}

void print_beam_time_cost(Output& beam_output)
{
	cout << "Beam Time Cost: " << beam_output.time_cost << "s" << endl << endl;
}

int main()
{
	set_robot_arm(3, 3, 0, 1, 0, 1, 2, 2);
	Output beam_output = beam_search(50, 1, 0.1, 0.01, false);
	print_beam_base_path();
	print_beam_joint_path();
	print_beam_distance_cost(beam_output);
	print_beam_time_cost(beam_output);

    return 0;
}