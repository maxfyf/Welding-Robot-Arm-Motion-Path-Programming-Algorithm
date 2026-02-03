#include <iostream>
#include "log.h"
#include "RRT.h"
using namespace std;

void print_beam_distance_cost(Beam_Output& beam_output)
{
	if (beam_output.success)
		cout << "Beam Distance Cost: " << ((int)(beam_output.distance_cost * 100 + 0.5)) / 100.0 << endl << endl;
	else
		cout << "Failed" << endl << endl;
}

void print_beam_base_path()
{
	if (!beam_base_path.size())
	{
		cout << "Failed" << endl << endl;
		return;
	}
	cout << "Beam Base Path:" << endl;
	for (auto& p : beam_base_path)
		cout << "(" << ((int)(p.x * 100 + 0.5)) / 100.0 << ", " << ((int)(p.y * 100 + 0.5)) / 100.0 << ")" << endl;
	cout << endl;
}

void print_beam_joint_path()
{
	if (!beam_joint_path.size())
	{
		cout << "Failed" << endl << endl;
		return;
	}
	cout << "Beam Joint Path:" << endl;
	for (auto& p : beam_joint_path)
		cout << "(" << ((int)(p.x * 100 + 0.5)) / 100.0 << ", " << ((int)(p.y * 100 + 0.5)) / 100.0 << ", " << ((int)(p.z * 100 + 0.5)) / 100.0 << ")" << endl;
	cout << endl;
}

void print_beam_path()
{
	if (!beam_base_path.size() || !beam_joint_path.size())
	{
		cout << "Failed" << endl << endl;
		return;
	}
	cout << "Beam Path:" << endl;
	for (int i = 0; i < beam_joint_path.size(); i++)
	{
		cout << "Base: (" << ((int)(beam_base_path[i].x * 100 + 0.5)) / 100.0 << ", " << ((int)(beam_base_path[i].y * 100 + 0.5)) / 100.0 << ") ";
		cout << "Joint: (" << ((int)(beam_joint_path[i].x * 100 + 0.5)) / 100.0 << ", " << ((int)(beam_joint_path[i].y * 100 + 0.5)) / 100.0 << ", " << ((int)(beam_joint_path[i].z * 100 + 0.5)) / 100.0 << ")" << endl;
	}
	cout << endl;
}

void print_beam_time_cost(Beam_Output& beam_output)
{
	if(beam_output.success)
	    cout << "Beam Time Cost: " << beam_output.time_cost << "s" << endl << endl;
	else
		cout << "Failed" << endl << endl;
}

void demonstrate_beam_results(Beam_Output& beam_output)
{
	if (beam_output.success)
	{
		print_beam_path();
		print_beam_distance_cost(beam_output);
		print_beam_time_cost(beam_output);
	}
	else
		cout << "Failed" << endl << endl;
}

void print_beam_cost(Beam_Output& beam_output)
{
	if(beam_output.success)
	{
		cout << "Beam Search: distance = " << ((int)(beam_output.distance_cost * 100 + 0.5)) / 100.0 << ", ";
		cout << "time = " << beam_output.time_cost << "s" << endl << endl;
	}
	else
		cout << "Failed" << endl << endl;
}

pair<double, double> print_rrt_success_rate(vector<RRT_Output*>& rrt_outputs)
{
	double total_time = 0;
	int success_count = 0;
	for (auto& output : rrt_outputs)
	{
		if (output->success)
		{
			success_count++;
			total_time += output->time_cost;
		}
	}
	double success_rate = ((int)((success_count * 10000 + 0.5) / rrt_outputs.size())) / 100.0;
	double average_time = success_count ? ((int)(total_time / success_count * 1000 + 0.5) / 1000.0) : 0;
	cout << "RRT Success Rate: " << success_rate << "%, " << "Average Time Cost: " << average_time << "s" << endl << endl;
	return make_pair(success_rate, average_time);
}

void print_rrt_end_distance_cost(RRT_Output& rrt_output)
{
	if(rrt_output.success)
	    cout << "RRT End Distance Cost: " << ((int)(rrt_output.end_distance_cost * 100 + 0.5)) / 100.0 << endl << endl;
	else
		cout << "Failed" << endl << endl;
}

void print_rrt_average_end_distance_cost(vector<RRT_Output*>& rrt_outputs)
{
	double total_distance = 0;
	int success_count = 0;
	for (auto& output : rrt_outputs)
	{
		if (output->success)
		{
			total_distance += output->end_distance_cost;
			success_count++;
		}
	}
	if (success_count > (rrt_outputs.size() >> 1))
		cout << "RRT Average End Distance Cost: " << ((int)((total_distance / success_count) * 100 + 0.5)) / 100.0 << endl << endl;
	else
		cout << "Failed" << endl << endl;
}

void print_rrt_distance_cost(RRT_Output& rrt_output)
{
	if(rrt_output.success)
    	cout << "RRT Distance Cost: " << ((int)(rrt_output.distance_cost * 100 + 0.5)) / 100.0 << endl << endl;
	else
		cout << "Failed" << endl << endl;
}

double print_rrt_average_distance_cost(vector<RRT_Output*>& rrt_outputs)
{
	double total_distance = 0;
	int success_count = 0;
	for (auto& output : rrt_outputs)
	{
		if (output->success)
		{
			total_distance += output->distance_cost;
			success_count++;
		}
	}
	double average_distance = 0;
	if (success_count > (rrt_outputs.size() >> 1))
	{
		average_distance = ((int)((total_distance / success_count) * 100 + 0.5)) / 100.0;
		cout << "RRT Average Distance Cost: " << average_distance << endl << endl;
	}
	else
		cout << "Failed" << endl << endl;
	return average_distance;
}

void print_rrt_base_path()
{
	if (!rrt_base_path.size())
	{
		cout << "Failed" << endl << endl;
		return;
	}
	cout << "RRT Base Path:" << endl;
	for (auto& p : rrt_base_path)
		cout << "(" << ((int)(p.x * 100 + 0.5)) / 100.0 << ", " << ((int)(p.y * 100 + 0.5)) / 100.0 << ")" << endl;
	cout << endl;
}

void print_rrt_joint_path()
{
	if (!rrt_joint_path.size())
	{
		cout << "Failed" << endl << endl;
		return;
	}
	cout << "RRT Joint Path : " << endl;
	for (auto& p : rrt_joint_path)
		cout << "(" << ((int)(p.x * 100 + 0.5)) / 100.0 << ", " << ((int)(p.y * 100 + 0.5)) / 100.0 << ", " << ((int)(p.z * 100 + 0.5)) / 100.0 << ")" << endl;
	cout << endl;
}

void print_rrt_end_path()
{
	if (!rrt_end_path.size())
	{
		cout << "Failed" << endl << endl;
		return;
	}
	cout << "RRT End Path:" << endl;
	for (auto& p : rrt_end_path)
		cout << "(" << ((int)(p.x * 100 + 0.5)) / 100.0 << ", " << ((int)(p.y * 100 + 0.5)) / 100.0 << ", " << ((int)(p.z * 100 + 0.5)) / 100.0 << ")" << endl;
	cout << endl;
}

void print_rrt_path()
{
	if (!rrt_base_path.size() || !rrt_joint_path.size())
	{
		cout << "Failed" << endl << endl;
		return;
	}
	cout << "RRT Path:" << endl;
	for (int i = 0; i < rrt_base_path.size(); i++)
	{
		cout << "Base: (" << ((int)(rrt_base_path[i].x * 100 + 0.5)) / 100.0 << ", " << ((int)(rrt_base_path[i].y * 100 + 0.5)) / 100.0 << ") ";
		cout << "Joint: (" << ((int)(rrt_joint_path[i].x * 100 + 0.5)) / 100.0 << ", " << ((int)(rrt_joint_path[i].y * 100 + 0.5)) / 100.0 << ", " << ((int)(rrt_joint_path[i].z * 100 + 0.5)) / 100.0 << ") " << endl;
	}
	cout << endl;
}

void print_rrt_time_cost(RRT_Output& rrt_output)
{
	if(rrt_output.success)
    	cout << "RRT Time Cost: " << rrt_output.time_cost << "s" << endl << endl;
	else
		cout << "Failed" << endl << endl;
}

double print_rrt_average_time_cost(vector<RRT_Output*>& rrt_outputs)
{
	double total_time = 0;
	int success_count = 0;
	for (auto& output : rrt_outputs)
	{
		if (output->success)
		{
			total_time += output->time_cost;
			success_count++;
		}
	}
	double average_time = 0;
	if (success_count > (rrt_outputs.size() >> 1))
	{
		average_time = (int)(total_time / success_count * 1000 + 0.5) / 1000.0;
		cout << "RRT Average Time Cost: " << average_time << "s" << endl << endl;
	}
	else
		cout << "Failed" << endl << endl;
	return average_time;
}

void demonstrate_rrt_results(RRT_Output& rrt_output)
{
	if(rrt_output.success)
	{
		print_rrt_end_path();
		print_rrt_path();
		print_rrt_end_distance_cost(rrt_output);
		print_rrt_distance_cost(rrt_output);
		print_rrt_time_cost(rrt_output);
	}
	else
		cout << "Failed" << endl << endl;
}

void print_rrt_cost(RRT_Output& rrt_output)
{
	if(rrt_output.success)
	{
		cout << "RRT Search: end distance = " << ((int)(rrt_output.end_distance_cost * 100 + 0.5)) / 100.0 << ", ";
		cout << "distance = " << ((int)(rrt_output.distance_cost * 100 + 0.5)) / 100.0 << ", ";
		cout << "time = " << rrt_output.time_cost << "s" << endl << endl;
	}
	else
		cout << "Failed" << endl << endl;
}

RRT_Output print_rrt_average_cost(vector<RRT_Output*>& rrt_outputs)
{
	double total_end_distance = 0;
	double total_distance = 0;
	double total_time = 0;
	int success_count = 0;
	for (auto& output : rrt_outputs)
	{
		if (output->success)
		{
			total_end_distance += output->end_distance_cost;
			total_distance += output->distance_cost;
			total_time += output->time_cost;
			success_count++;
		}
	}
	RRT_Output rrt_output = { false, 0, 0, 0 };
	if (success_count > (rrt_outputs.size() >> 1))
	{
		double average_end_distance = ((int)((total_end_distance / success_count) * 100 + 0.5)) / 100.0;
		double average_distance = ((int)((total_distance / success_count) * 100 + 0.5)) / 100.0;
		double average_time = (int)(total_time / success_count * 1000 + 0.5) / 1000.0;
		rrt_output = { true, average_end_distance, average_distance, average_time };
		cout << "RRT Average Search: end distance = " << average_end_distance << ", ";
		cout << "distance = " << average_distance << ", ";
		cout << "time = " << average_time << "s" << endl << endl;
	}
	else
		cout << "Failed" << endl << endl;
	return rrt_output;
}