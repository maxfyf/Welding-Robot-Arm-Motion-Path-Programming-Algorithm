#include <iostream>
#include <vector>
#include "global.h"
#include "Beam.h"
#include "RRT.h"
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

void print_rrt_distance_cost(Output& rrt_output)
{
	cout << "RRT Distance Cost: " << rrt_output.distance_cost << endl << endl;
}

void print_rrt_base_path()
{
	cout << "RRT Base Path:" << endl;
	for (auto& p : rrt_base_path)
		cout << "(" << ((int)(p.x * 100)) / 100.0 << ", " << ((int)(p.y * 100)) / 100.0 << ")" << endl;
	cout << endl;
}

void print_rrt_joint_path()
{
	cout << "RRT Joint Path : " << endl;
	for (auto& p : rrt_joint_path)
		cout << "(" << ((int)(p.x * 100)) / 100.0 << ", " << ((int)(p.y * 100)) / 100.0 << ", " << ((int)(p.z * 100)) / 100.0 << ")" << endl;
	cout << endl;
}

void print_rrt_end_path()
{
	cout << "RRT End Path:" << endl;
	for (auto& p : rrt_end_path)
		cout << "(" << ((int)(p.x * 100)) / 100.0 << ", " << ((int)(p.y * 100)) / 100.0 << ")" << endl;
	cout << endl;
}

void print_rrt_time_cost(Output& rrt_output)
{
	cout << "RRT Time Cost: " << rrt_output.time_cost << "s" << endl << endl;
}

int main()
{
	set_robot_arm(3, 3, 0, 2, 0, 2, 10, 2);

	/*Output beam_output = beam_search(50, 1, 0.1, 0.01, true);
	print_beam_base_path();
	print_beam_joint_path();
	print_beam_distance_cost(beam_output);
	print_beam_time_cost(beam_output);*/

	Output rrt_output = RRT(0.2, 0.1, 2, 0.5, 1, false, false, false);
	//print_rrt_base_path();
	//print_rrt_joint_path();
	print_rrt_end_path();
	print_rrt_distance_cost(rrt_output);
	print_rrt_time_cost(rrt_output);

    return 0;
}