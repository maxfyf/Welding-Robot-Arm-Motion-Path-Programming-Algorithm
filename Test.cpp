#include <iostream>
#include <vector>
#include "global.h"
#include "Beam.h"
using namespace std;

int main()
{
	init_robot_arm(3, 3, 0, 1, 0, 1, 2, 2);
	Output beam_output = beam_search(50, 0.04, 0.02, 0.1, 0.01, false);
	double cost = beam_output.distance_cost;
	cout << "Cost = " << cost << endl;
	cout << "Base Path:" << endl;
	for (auto &p : base_path)
		cout << "(" << ((int)(p.x * 100)) / 100.0 << ", " << ((int)(p.y * 100)) / 100.0 << ")" << endl;
	cout << "Joint Path:" << endl;
	for (auto &p : joint_path)
		cout << "(" << ((int)(p.x * 100)) / 100.0 << ", " << ((int)(p.y * 100)) / 100.0 << ", " << ((int)(p.z * 100)) / 100.0 << ")" << endl;
	cout << "Time Cost = " << beam_output.time_cost << "s" << endl;

    return 0;
}