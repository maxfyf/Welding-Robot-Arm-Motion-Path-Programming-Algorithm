#include <iostream>
#include <vector>
#include "global.h"
#include "Beam.h"
#include "RRT.h"
using namespace std;

void print_beam_distance_cost(Beam_Output& beam_output)
{
	cout << "Beam Distance Cost: " << ((int)(beam_output.distance_cost * 100)) / 100.0 << endl << endl;
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

void print_beam_path()
{
	cout << "Beam Path:" << endl;
	for (int i = 0; i < beam_joint_path.size(); i++)
	{
		cout << "Base: (" << ((int)(beam_base_path[i].x * 100)) / 100.0 << ", " << ((int)(beam_base_path[i].y * 100)) / 100.0 << ") ";
		cout << "Joint: (" << ((int)(beam_joint_path[i].x * 100)) / 100.0 << ", " << ((int)(beam_joint_path[i].y * 100)) / 100.0 << ", " << ((int)(beam_joint_path[i].z * 100)) / 100.0 << ")" << endl;
	}
	cout << endl;
}

void print_beam_time_cost(Beam_Output& beam_output)
{
	cout << "Beam Time Cost: " << beam_output.time_cost << "s" << endl << endl;
}

void print_rrt_end_distance_cost(RRT_Output& rrt_output)
{
	cout << "RRT End Distance Cost: " << ((int)(rrt_output.end_distance_cost * 100)) / 100.0 << endl << endl;
}

void print_rrt_distance_cost(RRT_Output& rrt_output)
{
	cout << "RRT Distance Cost: " << ((int)(rrt_output.distance_cost * 100)) / 100.0 << endl << endl;
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
		cout << "(" << ((int)(p.x * 100)) / 100.0 << ", " << ((int)(p.y * 100)) / 100.0 << ", " << ((int)(p.z * 100)) / 100.0 <<  ")" << endl;
	cout << endl;
}

void print_rrt_path()
{
	cout << "RRT Path:" << endl;
	for (int i = 0; i < rrt_base_path.size(); i++)
	{
		cout << "Base: (" << ((int)(rrt_base_path[i].x * 100)) / 100.0 << ", " << ((int)(rrt_base_path[i].y * 100)) / 100.0 << ") ";
		cout << "Joint: (" << ((int)(rrt_joint_path[i].x * 100)) / 100.0 << ", " << ((int)(rrt_joint_path[i].y * 100)) / 100.0 << ", " << ((int)(rrt_joint_path[i].z * 100)) / 100.0 << ") " << endl;
	}
	cout << endl;
}

void print_rrt_time_cost(RRT_Output& rrt_output)
{
	cout << "RRT Time Cost: " << rrt_output.time_cost << "s" << endl << endl;
}

int main()
{
	init_robot_arm(3, 3);
	set_base_position(0, 2);
	set_end_position(0, 2, 10, 2);

	if(0)
	{
		beam_config(50, 1, 0.1, 0.01, false);
		Beam_Output beam_output = beam_search();
		print_beam_path();
		print_beam_distance_cost(beam_output);
		print_beam_time_cost(beam_output);
		reset_beam();
	}
	else
	{
		beam_config(50, 1, 0.1, 0.01, false);
		rrt_config(0.2, 0.1, 2, 0.5, 1, false, false, false);
		RRT_Output rrt_output = RRT_search();
		print_rrt_end_path();
		print_rrt_path();
		print_rrt_end_distance_cost(rrt_output);
		print_rrt_distance_cost(rrt_output);
		print_rrt_time_cost(rrt_output);
		reset_rrt();
	}

    return 0;
}