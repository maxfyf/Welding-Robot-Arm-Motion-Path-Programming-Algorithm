#include "global.h"
#include "log.h"
#include "Beam.h"
#include "RRT.h"
#include "matplotlibcpp.h"
#include <iostream>
#include <fstream>
using namespace std;
using namespace matplotlibcpp;

Beam_Output beam_output;
RRT_Output rrt_output;
vector<RRT_Output*> rrt_outputs;
Beam_Output total_output;
vector<Beam_Output*> total_outputs;

void Beam_test()
{
	init_robot_arm(3, 3);
	set_base_position(0, 4);
	set_end_position(0, 2, 10, 2);
	beam_config(3, WeightModel::NONE, 0, 0.5, 0.05, false);
	beam_output = beam_search();
	demonstrate_beam_results(beam_output);
	reset_beam();
	reset_position();
}

void RRT_test()
{
	init_robot_arm(3, 3);
	set_base_position(0, 3);
	set_obstacles(0.5, 1);
	set_end_position(0, 4, 0, 2);
	beam_config(3, WeightModel::NONE, 0, 0.5, 0.05, false);
	rrt_config(1000, 1.0 / 5, 0.5, 1, 1, true, true, true, HeuristicFunction::NONE);
	rrt_output = RRT_search();
	demonstrate_rrt_results(rrt_output);
	reset_rrt();
	reset_position();
}

/*
================================
Beam Search Algorithm Experiment
================================
 */

void adjust_beam_width()
{
	init_robot_arm(3, 3);
	set_base_position(0, 4);
	set_end_position(0, 2, 10, 2);
	cout << "Basic settings: 3, 3; 0, 2; 0, 2, 10, 2" << endl << endl << endl;

	vector<double> x, y1, y2, y3, y4, z1, z2, z3, z4;
	x.reserve(20);
	y1.reserve(20);
	y2.reserve(20);
	y3.reserve(20);
	y4.reserve(20);
	z1.reserve(20);
	z2.reserve(20);
	z3.reserve(20);
	z4.reserve(20);
	for (int k = 5; k <= 100; k += 5)
	{
		cout << "Beam Width: " << k << endl;
		x.push_back(k);

		cout << "Step: " << 0.01 << endl;
		beam_config(k, WeightModel::NONE, 1, 0.1, 0.01, false);
		beam_output = beam_search();
		print_beam_cost(beam_output);
		if (beam_output.success)
		{
			y1.push_back(beam_output.distance_cost);
			z1.push_back(beam_output.time_cost);
		}
		else
		{
			y1.push_back(-1);
			z1.push_back(-1);
		}
		reset_beam();

		cout << "Step: " << 0.02 << endl;
		beam_config(k, WeightModel::NONE, 1, 0.2, 0.02, false);
		beam_output = beam_search();
		print_beam_cost(beam_output);
		if (beam_output.success)
		{
			y2.push_back(beam_output.distance_cost);
			z2.push_back(beam_output.time_cost);
		}
		else
		{
			y2.push_back(-1);
			z2.push_back(-1);
		}
		reset_beam();

		cout << "Step: " << 0.05 << endl;
		beam_config(k, WeightModel::NONE, 1, 0.5, 0.05, false);
		beam_output = beam_search();
		print_beam_cost(beam_output);
		if (beam_output.success)
		{
			y3.push_back(beam_output.distance_cost);
			z3.push_back(beam_output.time_cost);
		}
		else
		{
			y3.push_back(-1);
			z3.push_back(-1);
		}
		reset_beam();

		cout << "Step: " << 0.1 << endl;
		beam_config(k, WeightModel::NONE, 1, 1.0, 0.1, false);
		beam_output = beam_search();
		print_beam_cost(beam_output);
		if (beam_output.success)
		{
			y4.push_back(beam_output.distance_cost);
			z4.push_back(beam_output.time_cost);
		}
		else
		{
			y4.push_back(-1);
			z4.push_back(-1);
		}
		reset_beam();

		cout << endl << "----------------------------------------" << endl << endl;
	}

	figure_size(1200, 800);
	plot(x, y1, { {"label", "Step = 0.01"}, {"color", "red"}, {"marker", "o"}});
	plot(x, y2, { {"label", "Step = 0.02"}, {"color", "yellow"}, {"marker", "s"}});
	plot(x, y3, { {"label", "Step = 0.05"}, {"color", "green"}, {"marker", "^"}});
	plot(x, y4, { {"label", "Step = 0.1"}, {"color", "blue"}, {"marker", "d"}});
	title("Adjust Beam Width", { {"fontweight", "bold"}, {"fontsize", "16"}});
	xlabel("Beam Width", { {"fontweight", "bold"}});
	ylabel("Distance Cost", { {"fontweight", "bold"}});
	ylim(13, 14);
	legend();
	grid(true);
	save("DistanceCost-BeamWidth.png");
	close();

	figure_size(1200, 800);
	plot(x, z1, { {"label", "Step = 0.01"}, {"color", "red"}, {"marker", "o"} });
	plot(x, z2, { {"label", "Step = 0.02"}, {"color", "yellow"}, {"marker", "s"} });
	plot(x, z3, { {"label", "Step = 0.05"}, {"color", "green"}, {"marker", "^"} });
	plot(x, z4, { {"label", "Step = 0.1"}, {"color", "blue"}, {"marker", "d"} });
	title("Adjust Beam Width", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Beam Width", { {"fontweight", "bold"} });
	ylabel("Time Cost/s", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("TimeCost-BeamWidth.png");
	close();
}

void adjust_beam_step()
{
	init_robot_arm(3, 3);

	int k = 5;
	cout << "Beam Width: " << k << endl << endl << endl;

	vector<double> x, y1, y2, y3, y4, z1, z2, z3, z4;
	x.reserve(10);
	y1.reserve(10);
	y2.reserve(10);
	y3.reserve(10);
	y4.reserve(10);
	z1.reserve(10);
	z2.reserve(10);
	z3.reserve(10);
	z4.reserve(10);
	for (double step = 0.01; step <= 0.1; step += 0.01)
	{
		cout << "Step: " << step << endl;
		x.push_back(step);

		set_base_position(0, 2);
		set_end_position(0, 2, 10, 2);
		cout << "Basic settings: 3, 3; 0, 2; 0, 2, 10, 2" << endl;
		beam_config(k, WeightModel::NONE, 1, 10 * step, step, false);
		beam_output = beam_search();
		print_beam_cost(beam_output);
		if (beam_output.success)
		{
			y1.push_back(beam_output.distance_cost);
			z1.push_back(beam_output.time_cost);
		}
		else
		{
			y1.push_back(-1);
			z1.push_back(-1);
		}
		reset_beam();

		set_base_position(0, 4);
		set_end_position(0, 2, 10, 2);
		cout << "Basic settings: 3, 3; 0, 4; 0, 2, 10, 2" << endl;
		beam_config(k, WeightModel::NONE, 1, 10 * step, step, false);
		beam_output = beam_search();
		print_beam_cost(beam_output);
		if (beam_output.success)
		{
			y2.push_back(beam_output.distance_cost);
			z2.push_back(beam_output.time_cost);
		}
		else
		{
			y2.push_back(-1);
			z2.push_back(-1);
		}
		reset_beam();

		set_base_position(0, 2);
		set_end_position(0, 4, 10, 4);
		cout << "Basic settings: 3, 3; 0, 2; 0, 4, 10, 4" << endl;
		beam_config(k, WeightModel::NONE, 1, 10 * step, step, false);
		beam_output = beam_search();
		print_beam_cost(beam_output);
		if (beam_output.success)
		{
			y3.push_back(beam_output.distance_cost);
			z3.push_back(beam_output.time_cost);
		}
		else
		{
			y3.push_back(-1);
			z3.push_back(-1);
		}
		reset_beam();

		set_base_position(0, 4);
		set_end_position(0, 4, 10, 4);
		cout << "Basic settings: 3, 3; 0, 4; 0, 4, 10, 4" << endl;
		beam_config(k, WeightModel::NONE, 1, 10 * step, step, false);
		beam_output = beam_search();
		print_beam_cost(beam_output);
		if (beam_output.success)
		{
			y4.push_back(beam_output.distance_cost);
			z4.push_back(beam_output.time_cost);
		}
		else
		{
			y4.push_back(-1);
			z4.push_back(-1);
		}
		reset_beam();

		cout << endl << "----------------------------------------" << endl << endl;
	}

	figure_size(1200, 800);
	plot(x, y1, { {"label", "x0 = 2, z = 2"}, {"color", "red"}, {"marker", "o"}});
	plot(x, y2, { {"label", "x0 = 4, z = 2"}, {"color", "yellow"}, {"marker", "s"} });
	plot(x, y3, { {"label", "x0 = 2, z = 4"}, {"color", "green"}, {"marker", "^"} });
	plot(x, y4, { {"label", "x0 = 4, z = 4"}, {"color", "blue"}, {"marker", "d"} });
	title("Adjust Beam Step (Beam Width = 5)", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Beam Step", { {"fontweight", "bold"}});
	ylabel("Distance Cost", { {"fontweight", "bold"}});
	legend();
	grid(true);
	save("DistanceCost-BeamStep.png");
	close();

	figure_size(1200, 800);
	plot(x, z1, { {"label", "x0 = 2, z = 2"}, {"color", "red"}, {"marker", "s"} });
	plot(x, z2, { {"label", "x0 = 4, z = 2"}, {"color", "yellow"}, {"marker", "o"} });
	plot(x, z3, { {"label", "x0 = 2, z = 4"}, {"color", "green"}, {"marker", "^"} });
	plot(x, z4, { {"label", "x0 = 4, z = 4"}, {"color", "blue"}, {"marker", "d"} });
	title("Adjust Beam Step (Beam Width = 5)", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Beam Step", { {"fontweight", "bold"} });
	ylabel("Time Cost", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("TimeCost-BeamStep.png");
	close();
}

void primary_Beam_experiment()
{
	init_robot_arm(3, 3);
	int k = 5;
	double step = 0.07;
	beam_config(5, WeightModel::NONE, 1, 0.7, 0.07, false);
	cout << "Beam Width: " << k << ", Step: " << step << endl << endl << endl;

	vector<double> x, y1, y2, y3, y4, z1, z2, z3, z4;
	x.reserve(10);
	y1.reserve(10);
	y2.reserve(10);
	y3.reserve(10);
	y4.reserve(10);
	z1.reserve(10);
	z2.reserve(10);
	z3.reserve(10);
	z4.reserve(10);
	for (double w = 0; w <= 2; w += 0.1)
	{
		cout << "Weight: " << w << endl;
		x.push_back(w);

		set_base_position(0, 2);
		set_end_position(0, 2, 10, 2);
		cout << "Basic settings: 3, 3; 0, 2; 0, 2, 10, 2" << endl;
		beam_config(k, WeightModel::NONE, w, 10 * step, step, true);
		beam_output = beam_search();
		print_beam_cost(beam_output);
		if (beam_output.success)
		{
			y1.push_back(beam_output.distance_cost);
			z1.push_back(beam_output.time_cost);
		}
		else
		{
			y1.push_back(-1);
			z1.push_back(-1);
		}
		reset_beam();

		set_base_position(0, 4);
		set_end_position(0, 2, 10, 2);
		cout << "Basic settings: 3, 3; 0, 4; 0, 2, 10, 2" << endl;
		beam_config(k, WeightModel::NONE, w, 10 * step, step, true);
		beam_output = beam_search();
		print_beam_cost(beam_output);
		if (beam_output.success)
		{
			y2.push_back(beam_output.distance_cost);
			z2.push_back(beam_output.time_cost);
		}
		else
		{
			y2.push_back(-1);
			z2.push_back(-1);
		}
		reset_beam();

		set_base_position(0, 2);
		set_end_position(0, 4, 10, 4);
		cout << "Basic settings: 3, 3; 0, 2; 0, 4, 10, 4" << endl;
		beam_config(k, WeightModel::NONE, w, 10 * step, step, true);
		beam_output = beam_search();
		print_beam_cost(beam_output);
		if (beam_output.success)
		{
			y3.push_back(beam_output.distance_cost);
			z3.push_back(beam_output.time_cost);
		}
		else
		{
			y3.push_back(-1);
			z3.push_back(-1);
		}
		reset_beam();

		set_base_position(0, 4);
		set_end_position(0, 4, 10, 4);
		cout << "Basic settings: 3, 3; 0, 4; 0, 4, 10, 4" << endl;
		beam_config(k, WeightModel::NONE, w, 10 * step, step, true);
		beam_output = beam_search();
		print_beam_cost(beam_output);
		if (beam_output.success)
		{
			y4.push_back(beam_output.distance_cost);
			z4.push_back(beam_output.time_cost);
		}
		else
		{
			y4.push_back(-1);
			z4.push_back(-1);
		}
		reset_beam();

		cout << endl << "----------------------------------------" << endl << endl;
	}

	figure_size(1200, 800);
	plot(x, y1, { {"label", "x0 = 2, z = 2"}, {"color", "red"}, {"marker", "o"} });
	plot(x, y2, { {"label", "x0 = 4, z = 2"}, {"color", "yellow"}, {"marker", "s"} });
	plot(x, y3, { {"label", "x0 = 2, z = 4"}, {"color", "green"}, {"marker", "^"} });
	plot(x, y4, { {"label", "x0 = 4, z = 4"}, {"color", "blue"}, {"marker", "d"} });
	title("Primary Beam Experiment (Beam Width = 5, Beam Step = 0.07)", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Weight", { {"fontweight", "bold"} });
	ylabel("Distance Cost", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("DistanceCost-Weight.png");
	close();

	figure_size(1200, 800);
	plot(x, z1, { {"label", "x0 = 2, z = 2"}, {"color", "red"}, {"marker", "s"} });
	plot(x, z2, { {"label", "x0 = 4, z = 2"}, {"color", "yellow"}, {"marker", "o"} });
	plot(x, z3, { {"label", "x0 = 2, z = 4"}, {"color", "green"}, {"marker", "^"} });
	plot(x, z4, { {"label", "x0 = 4, z = 4"}, {"color", "blue"}, {"marker", "d"} });
	title("Primary Beam Experiment (Beam Width = 5, Beam Step = 0.07)", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Weight", { {"fontweight", "bold"} });
	ylabel("Time Cost", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("TimeCost-Weight.png");
	close();
}

void Beam_experiment()
{
	init_robot_arm(3, 3);
	int k = 5;
	double step = 0.07;
	beam_config(k, WeightModel::NONE, 1, 10 * step, step, false);
	cout << "Beam Width: " << k << ", Step: " << step << endl << endl << endl;
	ofstream file("Beam_experiment.csv");
	file << "Beam Experiment (Beam Width = " << k << " Step = " << step << ")\n";

	for (double x = 0.5; x < 6; x += 0.5)
	{
		for (double z = 0.5; z < 6 && x * x + z * z < 36; z += 0.5)
		{
			set_base_position(0, x);
			set_end_position(0, z, 10, z);
			cout << "Basic settings: 3, 3; 0, " << x << "; 0, " << z << ", 10, " << z << endl;
			file << "Basic settings: 3 3; 0 " << x << "; 0 " << z << " 10 " << z << "\n";
			file << "Weight, Distance Cost\n";

			for (double w = 0; w <= 0.5; w += 0.1)
			{
				cout << "Weight: " << w << endl;
				beam_config(k, WeightModel::NONE, w, 10 * step, step, true);
				beam_output = beam_search();
				print_beam_cost(beam_output);
				file << w << ", " << beam_output.distance_cost << "\n";
				reset_beam();
				cout << endl;
			}
			cout << endl << "-------------------------------------------------------" << endl << endl;
		}
	}
}

void adjust_beam_weight_model()
{
	double optimization_ratio;
	Beam_Output baseline, model;
	WeightModel wm;
	int cnt[5];
	memset(cnt, 0, 5 * sizeof(int));
	ofstream file("Beam权重模型实验结果.csv");
	file << "实验配置：机械臂(3 3)，基座y坐标与末端z始末坐标服从N(3 1)，末端x坐标增大10，共100组实验" << endl << endl;
	file << "优化率表(%)" << endl;
	file << "测试序号, w-z线性模型, w-z指数模型, w-θ线性模型, w-θ对数模型, 最优模型" << endl;

	init_robot_arm(3, 3);
	cout << "Weight Test" << endl;
	int k = 5;
	double step = 0.07;
	cout << "Beam Width: " << k << ", Step: " << step << endl << endl << endl;
	for (int i = 0; i < 100; i++)
	{
		set_random_case();
		cout << "Test Case " << i + 1 << ":" << endl;
		file << i + 1 << ", ";
		cout << "Base Position: (" << x_b1 << ", " << y_b1 << ", 0), End Position: (" << x_e1 << ", 0, " << z_e1 << ") to (" << x_e2 << ", 0, " << z_e2 << ")" << endl;

		beam_config(k, WeightModel::NONE, 0, 10 * step, step, true);
		baseline = beam_search();
		cout << "Baseline: Cost = " << baseline.distance_cost << endl;
		reset_beam();

		model = baseline;
		wm = WeightModel::NONE;

		beam_config(k, WeightModel::Z_LINEAR, 0, 10 * step, step, true); 
		beam_output = beam_search();
		optimization_ratio = (baseline.distance_cost - beam_output.distance_cost) / baseline.distance_cost * 100;
		file << optimization_ratio << ", ";
		cout << "z linear model: Cost = " << beam_output.distance_cost << ", Optimization Ratio = " << optimization_ratio << "%" << endl;
		if(beam_output.success && beam_output.distance_cost <= model.distance_cost)
		{
			model = beam_output;
			wm = WeightModel::Z_LINEAR;
		}
		reset_beam();

		beam_config(k, WeightModel::Z_EXPONENTIAL, 0, 10 * step, step, true);
		beam_output = beam_search();
		optimization_ratio = (baseline.distance_cost - beam_output.distance_cost) / baseline.distance_cost * 100;
		file << optimization_ratio << ", ";
		cout << "z exponential model: Cost = " << beam_output.distance_cost << ", Optimization Ratio = " << optimization_ratio << "%" << endl;
		if (beam_output.success && beam_output.distance_cost <= model.distance_cost)
		{
			model = beam_output;
			wm = WeightModel::Z_EXPONENTIAL;
		}
		reset_beam();

		beam_config(k, WeightModel::THETA_LINEAR, 0, 10 * step, step, true);
		beam_output = beam_search();
		optimization_ratio = (baseline.distance_cost - beam_output.distance_cost) / baseline.distance_cost * 100;
		file << optimization_ratio << ", ";
		cout << "theta linear model: Cost = " << beam_output.distance_cost << ", Optimization Ratio = " << optimization_ratio << "%" << endl;
		if (beam_output.success && beam_output.distance_cost <= model.distance_cost)
		{
			model = beam_output;
			wm = WeightModel::THETA_LINEAR;
		}
		reset_beam();

		beam_config(k, WeightModel::THETA_LOGARITHMIC, 0, 10 * step, step, true);
		beam_output = beam_search();
		optimization_ratio = (baseline.distance_cost - beam_output.distance_cost) / baseline.distance_cost * 100;
		file << optimization_ratio << ", ";
		cout << "theta logarithmic model: Cost = " << beam_output.distance_cost << ", Optimization Ratio = " << optimization_ratio << "%" << endl;
		if (beam_output.success && beam_output.distance_cost <= model.distance_cost)
		{
			model = beam_output;
			wm = WeightModel::THETA_LOGARITHMIC;
		}
		reset_beam();

		cout << endl;
		if (wm == WeightModel::NONE)
		{
			cout << "Best Model: no optimization" << endl << endl;
			file << "无优化" << endl;
			cout << "Baseline: " << endl;
			print_beam_cost(baseline);
		}
		else
		{
			cout << "Best Model: ";
			switch (wm)
			{
			case WeightModel::Z_LINEAR:
				cout << "z linear model" << endl;
				file << "w-z线性模型" << endl;
				break;
			case WeightModel::Z_EXPONENTIAL:
				cout << "z exponential model" << endl;
				file << "w-z指数模型" << endl;
				break;
			case WeightModel::THETA_LINEAR:
				cout << "theta linear model" << endl;
				file << "w-θ线性模型" << endl;
				break;
			case WeightModel::THETA_LOGARITHMIC:
				cout << "theta logarithmic model" << endl;
				file << "w-θ对数模型" << endl;
				break;
			}
			cout << "Optimized Result: " << endl;
			print_beam_cost(model);
			cout << "Baseline: " << endl;
			print_beam_cost(baseline);
		}
		cnt[(int)wm]++;
		cout << "----------------------------------------------------------------------------" << endl << endl;
	}

	cout << "Best Model Count:" << endl;
	cout << "no optimization: " << cnt[0] << endl;
	cout << "z linear model: " << cnt[1] << endl;
	cout << "z exponential model: " << cnt[2] << endl;
	cout << "theta linear model: " << cnt[3] << endl;
	cout << "theta logarithmic model: " << cnt[4] << endl;
	file << endl;
	file << "最优模型计数" << endl;
	file << "模型, 计数" << endl;
	file << "无优化, " << cnt[0] << endl;
	file << "w-z线性模型, " << cnt[1] << endl;
	file << "w-z指数模型, " << cnt[2] << endl;
	file << "w-θ线性模型, " << cnt[3] << endl;
	file << "w-θ对数模型, " << cnt[4] << endl;
}

/*
========================================================
Rapidly-exploring Random Tree (RRT) Algorithm Experiment
========================================================
 */

void RRT_goal_bias_experiment()
{
	init_robot_arm(3, 3);
	set_base_position(0, 3);
	set_end_position(0, 4.5, 0, 1.5);
	cout << "Basic settings: 3, 3; 0, 3; 0, 4.5, 0, 1.5" << endl;
	beam_config(5, WeightModel::THETA_LINEAR, 0, 0.7, 0.07, true);
	cout << "Beam Algorithm settings: 5, 0.07, theta linear model" << endl << endl << endl;

	pair<double, double> p;
	vector<double> x, r1, r2, r3, r4, r5, r6, t1, t2, t3, t4, t5, t6;
	x.reserve(20);
	r1.reserve(20);
	r2.reserve(20);
	r3.reserve(20);
	r4.reserve(20);
	r5.reserve(20);
	r6.reserve(20);
	t1.reserve(20);
	t2.reserve(20);
	t3.reserve(20);
	t4.reserve(20);
	t5.reserve(20);
	t6.reserve(20);

	for (double i = 0; i < 1; i += 0.05)
	{
		cout << "Goal Bias Ratio: " << i << endl;
		x.push_back(i);
		rrt_config(1000, 1.0 / 5, i, 0, 0, true, false, false, HeuristicFunction::NONE);

		set_obstacles(0.33, 0.5);
		cout << "Obstacle settings: 0.33, 0.5" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		p = print_rrt_success_rate(rrt_outputs);
		r1.push_back(p.first);
		t1.push_back(p.second);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.33, 1);
		cout << "Obstacle settings: 0.33, 1" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		p = print_rrt_success_rate(rrt_outputs);
		r2.push_back(p.first);
		t2.push_back(p.second);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.33, 2);
		cout << "Obstacle settings: 0.33, 2" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		p = print_rrt_success_rate(rrt_outputs);
		r3.push_back(p.first);
		t3.push_back(p.second);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 0.5);
		cout << "Obstacle settings: 0.67, 0.5" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		p = print_rrt_success_rate(rrt_outputs);
		r4.push_back(p.first);
		t4.push_back(p.second);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 1);
		cout << "Obstacle settings: 0.67, 1" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		p = print_rrt_success_rate(rrt_outputs);
		r5.push_back(p.first);
		t5.push_back(p.second);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 2);
		cout << "Obstacle settings: 0.67, 2" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		p = print_rrt_success_rate(rrt_outputs);
		r6.push_back(p.first);
		t6.push_back(p.second);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		cout << endl << "----------------------------------------" << endl << endl;
	}

	figure_size(1200, 800);
	plot(x, r1, { {"label", "Obstacle settings: 0.33, 0.5"}, {"color", "red"}, {"marker", "o"} });
	plot(x, r2, { {"label", "Obstacle settings: 0.33, 1"}, {"color", "orange"}, {"marker", "s"} });
	plot(x, r3, { {"label", "Obstacle settings: 0.33, 2"}, {"color", "yellow"}, {"marker", "^"} });
	plot(x, r4, { {"label", "Obstacle settings: 0.67, 0.5"}, {"color", "green"}, {"marker", "d"} });
	plot(x, r5, { {"label", "Obstacle settings: 0.67, 1"}, {"color", "blue"}, {"marker", "p"} });
	plot(x, r6, { {"label", "Obstacle settings: 0.67, 2"}, {"color", "violet"}, {"marker", "h"} });
	title("Goal Bias Experiment", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Goal Bias Ratio", { {"fontweight", "bold"} });
	ylabel("Success Rate/%", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("SuccessRate-GoalBiasRatio.png");
	close();

	figure_size(1200, 800);
	plot(x, t1, { {"label", "Obstacle settings: 0.33, 0.5"}, {"color", "red"}, {"marker", "o"} });
	plot(x, t2, { {"label", "Obstacle settings: 0.33, 1"}, {"color", "orange"}, {"marker", "s"} });
	plot(x, t3, { {"label", "Obstacle settings: 0.33, 2"}, {"color", "yellow"}, {"marker", "^"} });
	plot(x, t4, { {"label", "Obstacle settings: 0.67, 0.5"}, {"color", "green"}, {"marker", "d"} });
	plot(x, t5, { {"label", "Obstacle settings: 0.67, 1"}, {"color", "blue"}, {"marker", "p"} });
	plot(x, t6, { {"label", "Obstacle settings: 0.67, 2"}, {"color", "violet"}, {"marker", "h"} });
	title("Goal Bias Experiment", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Goal Bias Ratio", { {"fontweight", "bold"} });
	ylabel("Average Time Cost/s", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageTimeCost-GoalBiasRatio.png");
	close();
}

void adjust_rrt_step_ratio()
{
	init_robot_arm(3, 3);
	set_base_position(0, 3);
	set_end_position(0, 4.5, 0, 1.5);
	cout << "Basic settings: 3, 3; 0, 3; 0, 4.5, 0, 1.5" << endl;
	beam_config(5, WeightModel::THETA_LINEAR, 0, 0.7, 0.07, true);
	cout << "Beam Algorithm settings: 5, 0.07, theta linear model" << endl;
	cout << "RRT Goal Bias Ratio: 0.3" << endl << endl << endl;

	vector<double> x, c1, c2, c3, c4, c5, c6, y1, y2, y3, y4, y5, y6, t1, t2, t3, t4, t5, t6;
	x.reserve(18);
	c1.reserve(18);
	c2.reserve(18);
	c3.reserve(18);
	c4.reserve(18);
	c5.reserve(18);
	c6.reserve(18);
	y1.reserve(18);
	y2.reserve(18);
	y3.reserve(18);
	y4.reserve(18);
	y5.reserve(18);
	y6.reserve(18);
	t1.reserve(18);
	t2.reserve(18);
	t3.reserve(18);
	t4.reserve(18);
	t5.reserve(18);
	t6.reserve(18);

	for (int i = 3; i <= 20; i++)
	{
		cout << "RRT Step Ratio: 1/" << i << endl;
		x.push_back(i);
		rrt_config(1000, 1.0 / i, 0.3, 0, 0, true, false, false, HeuristicFunction::NONE);

		set_obstacles(0.33, 0.5);
		cout << "Obstacle settings: 0.33, 0.5" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c1.push_back(rrt_output.end_distance_cost);
		y1.push_back(rrt_output.distance_cost);
		t1.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.33, 1);
		cout << "Obstacle settings: 0.33, 1" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c2.push_back(rrt_output.end_distance_cost);
		y2.push_back(rrt_output.distance_cost);
		t2.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.33, 2);
		cout << "Obstacle settings: 0.33, 2" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c3.push_back(rrt_output.end_distance_cost);
		y3.push_back(rrt_output.distance_cost);
		t3.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 0.5);
		cout << "Obstacle settings: 0.67, 0.5" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c4.push_back(rrt_output.end_distance_cost);
		y4.push_back(rrt_output.distance_cost);
		t4.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 1);
		cout << "Obstacle settings: 0.67, 1" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c5.push_back(rrt_output.end_distance_cost);
		y5.push_back(rrt_output.distance_cost);
		t5.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 2);
		cout << "Obstacle settings: 0.67, 2" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c6.push_back(rrt_output.end_distance_cost);
		y6.push_back(rrt_output.distance_cost);
		t6.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		cout << endl << "----------------------------------------" << endl << endl;
	}

	figure_size(1200, 800);
	plot(x, c1, { {"label", "Obstacle settings: 0.33, 0.5"}, {"color", "red"}, {"marker", "o"} });
	plot(x, c2, { {"label", "Obstacle settings: 0.33, 1"}, {"color", "orange"}, {"marker", "s"} });
	plot(x, c3, { {"label", "Obstacle settings: 0.33, 2"}, {"color", "yellow"}, {"marker", "^"} });
	plot(x, c4, { {"label", "Obstacle settings: 0.67, 0.5"}, {"color", "green"}, {"marker", "d"} });
	plot(x, c5, { {"label", "Obstacle settings: 0.67, 1"}, {"color", "blue"}, {"marker", "p"} });
	plot(x, c6, { {"label", "Obstacle settings: 0.67, 2"}, {"color", "violet"}, {"marker", "h"} });
	title("Adjust RRT Step Ratio", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("1 / RRT Step Ratio", { {"fontweight", "bold"} });
	ylabel("Average End Distance Cost", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageEndDistanceCost-RRTStepRatio.png");
	close();

	figure_size(1200, 800);
	plot(x, y1, { {"label", "Obstacle settings: 0.33, 0.5"}, {"color", "red"}, {"marker", "o"} });
	plot(x, y2, { {"label", "Obstacle settings: 0.33, 1"}, {"color", "orange"}, {"marker", "s"} });
	plot(x, y3, { {"label", "Obstacle settings: 0.33, 2"}, {"color", "yellow"}, {"marker", "^"} });
	plot(x, y4, { {"label", "Obstacle settings: 0.67, 0.5"}, {"color", "green"}, {"marker", "d"} });
	plot(x, y5, { {"label", "Obstacle settings: 0.67, 1"}, {"color", "blue"}, {"marker", "p"} });
	plot(x, y6, { {"label", "Obstacle settings: 0.67, 2"}, {"color", "violet"}, {"marker", "h"} });
	title("Adjust RRT Step Ratio", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("1 / RRT Step Ratio", { {"fontweight", "bold"} });
	ylabel("Average Distance Cost", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageDistanceCost-RRTStepRatio.png");
	close();

	figure_size(1200, 800);
	plot(x, t1, { {"label", "Obstacle settings: 0.33, 0.5"}, {"color", "red"}, {"marker", "o"} });
	plot(x, t2, { {"label", "Obstacle settings: 0.33, 1"}, {"color", "orange"}, {"marker", "s"} });
	plot(x, t3, { {"label", "Obstacle settings: 0.33, 2"}, {"color", "yellow"}, {"marker", "^"} });
	plot(x, t4, { {"label", "Obstacle settings: 0.67, 0.5"}, {"color", "green"}, {"marker", "d"} });
	plot(x, t5, { {"label", "Obstacle settings: 0.67, 1"}, {"color", "blue"}, {"marker", "p"} });
	plot(x, t6, { {"label", "Obstacle settings: 0.67, 2"}, {"color", "violet"}, {"marker", "h"} });
	title("Adjust RRT Step Ratio", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("1 / RRT Step Ratio", { {"fontweight", "bold"} });
	ylabel("Average Time Cost/s", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageTimeCost-RRTStepRatio.png");
	close();
}

void adjust_rrt_neighbor_range_ratio()
{
	init_robot_arm(3, 3);
	set_base_position(0, 3);
	set_end_position(0, 4.5, 0, 1.5);
	cout << "Basic settings: 3, 3; 0, 3; 0, 4.5, 0, 1.5" << endl;
	beam_config(5, WeightModel::THETA_LINEAR, 0, 0.7, 0.07, true);
	cout << "Beam Algorithm settings: 5, 0.07, theta linear model" << endl;
	cout << "RRT Goal Bias Ratio: 0.3; Step Ratio: 1/3" << endl << endl << endl;

	vector<double> x, c1, c2, c3, c4, c5, c6, y1, y2, y3, y4, y5, y6, t1, t2, t3, t4, t5, t6;
	x.reserve(30);
	c1.reserve(30);
	c2.reserve(30);
	c3.reserve(30);
	c4.reserve(30);
	c5.reserve(30);
	c6.reserve(30);
	y1.reserve(30);
	y2.reserve(30);
	y3.reserve(30);
	y4.reserve(30);
	y5.reserve(30);
	y6.reserve(30);
	t1.reserve(30);
	t2.reserve(30);
	t3.reserve(30);
	t4.reserve(30);
	t5.reserve(30);
	t6.reserve(30);
	for (double nrr = 0.1; nrr <= 3; nrr += 0.1)
	{
		cout << "Neighbor Range Ratio: " << nrr << endl;
		x.push_back(nrr);
		rrt_config(1000, 1.0 / 3, 0.3, nrr, 0, true, false, true, HeuristicFunction::NONE);

		set_obstacles(0.33, 0.5);
		cout << "Obstacle settings: 0.33, 0.5" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c1.push_back(rrt_output.end_distance_cost);
		y1.push_back(rrt_output.distance_cost);
		t1.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.33, 1);
		cout << "Obstacle settings: 0.33, 1" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c2.push_back(rrt_output.end_distance_cost);
		y2.push_back(rrt_output.distance_cost);
		t2.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.33, 2);
		cout << "Obstacle settings: 0.33, 2" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c3.push_back(rrt_output.end_distance_cost);
		y3.push_back(rrt_output.distance_cost);
		t3.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 0.5);
		cout << "Obstacle settings: 0.67, 0.5" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c4.push_back(rrt_output.end_distance_cost);
		y4.push_back(rrt_output.distance_cost);
		t4.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 1);
		cout << "Obstacle settings: 0.67, 1" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c5.push_back(rrt_output.end_distance_cost);
		y5.push_back(rrt_output.distance_cost);
		t5.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 2);
		cout << "Obstacle settings: 0.67, 2" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c6.push_back(rrt_output.end_distance_cost);
		y6.push_back(rrt_output.distance_cost);
		t6.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		cout << endl << "----------------------------------------" << endl << endl;
	}

	figure_size(1200, 800);
	plot(x, c1, { {"label", "Obstacle settings: 0.33, 0.5"}, {"color", "red"}, {"marker", "o"} });
	plot(x, c2, { {"label", "Obstacle settings: 0.33, 1"}, {"color", "orange"}, {"marker", "s"} });
	plot(x, c3, { {"label", "Obstacle settings: 0.33, 2"}, {"color", "yellow"}, {"marker", "^"} });
	plot(x, c4, { {"label", "Obstacle settings: 0.67, 0.5"}, {"color", "green"}, {"marker", "d"} });
	plot(x, c5, { {"label", "Obstacle settings: 0.67, 1"}, {"color", "blue"}, {"marker", "p"} });
	plot(x, c6, { {"label", "Obstacle settings: 0.67, 2"}, {"color", "violet"}, {"marker", "h"} });
	title("Adjust Neighbor Range Ratio", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Neighbor Range Ratio", { {"fontweight", "bold"} });
	ylabel("Average End Distance Cost", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageEndDistanceCost-RRTNeighborRangeRatio.png");
	close();

	figure_size(1200, 800);
	plot(x, y1, { {"label", "Obstacle settings: 0.33, 0.5"}, {"color", "red"}, {"marker", "o"} });
	plot(x, y2, { {"label", "Obstacle settings: 0.33, 1"}, {"color", "orange"}, {"marker", "s"} });
	plot(x, y3, { {"label", "Obstacle settings: 0.33, 2"}, {"color", "yellow"}, {"marker", "^"} });
	plot(x, y4, { {"label", "Obstacle settings: 0.67, 0.5"}, {"color", "green"}, {"marker", "d"} });
	plot(x, y5, { {"label", "Obstacle settings: 0.67, 1"}, {"color", "blue"}, {"marker", "p"} });
	plot(x, y6, { {"label", "Obstacle settings: 0.67, 2"}, {"color", "violet"}, {"marker", "h"} });
	title("Adjust Neighbor Range Ratio", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Neighbor Range Ratio", { {"fontweight", "bold"} });
	ylabel("Average Distance Cost", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageDistanceCost-RRTNeighborRangeRatio.png");
	close();

	figure_size(1200, 800);
	plot(x, t1, { {"label", "Obstacle settings: 0.33, 0.5"}, {"color", "red"}, {"marker", "o"} });
	plot(x, t2, { {"label", "Obstacle settings: 0.33, 1"}, {"color", "orange"}, {"marker", "s"} });
	plot(x, t3, { {"label", "Obstacle settings: 0.33, 2"}, {"color", "yellow"}, {"marker", "^"} });
	plot(x, t4, { {"label", "Obstacle settings: 0.67, 0.5"}, {"color", "green"}, {"marker", "d"} });
	plot(x, t5, { {"label", "Obstacle settings: 0.67, 1"}, {"color", "blue"}, {"marker", "p"} });
	plot(x, t6, { {"label", "Obstacle settings: 0.67, 2"}, {"color", "violet"}, {"marker", "h"} });
	title("Adjust Neighbor Range Ratio", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Neighbor Range Ratio", { {"fontweight", "bold"} });
	ylabel("Average Time Cost/s", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageTimeCost-RRTNeighborRangeRatio.png");
	close();
}

void relaxation_optimize_experiment()
{
	init_robot_arm(3, 3);
	set_base_position(0, 3);
	set_end_position(0, 4.5, 0, 1.5);
	cout << "Basic settings: 3, 3; 0, 3; 0, 4.5, 0, 1.5" << endl;
	set_obstacles(0.5, 1);
	cout << "Obstacle settings: 0.5, 1" << endl;
	beam_config(5, WeightModel::THETA_LINEAR, 0, 0.7, 0.07, true);
	cout << "Beam Algorithm settings: 5, 0.07, theta linear model" << endl;
	cout << "Iterations: 3000; RRT Goal Bias Ratio: 0.3; Step Ratio: 1/3; neighbor range ratio: 1" << endl << endl << endl;

	vector<double> x, c1, c2, c, y1, y2, t1, t2;
	x.reserve(10);
	c1.reserve(10);
	c2.reserve(10);
	y1.reserve(10);
	y2.reserve(10);
	t1.reserve(10);
	t2.reserve(10);
	for (int it = 1000; it <= 10000; it += 1000)
	{
		x.push_back(it);
		c.push_back(4);

		rrt_config(it, 1.0 / 3, 0.3, 1, 0, true, false, false, HeuristicFunction::NONE);
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c1.push_back(rrt_output.end_distance_cost);
		y1.push_back(rrt_output.distance_cost);
		t1.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		rrt_config(it, 1.0 / 3, 0.3, 1, 0, true, false, true, HeuristicFunction::NONE);
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c2.push_back(rrt_output.end_distance_cost);
		y2.push_back(rrt_output.distance_cost);
		t2.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		cout << endl << "----------------------------------------" << endl << endl;
	}

	figure_size(1200, 800);
	plot(x, c1, { {"label", "Baseline"}, {"linestyle", "--"}, {"color", "black"}, { "marker", "o" }});
	plot(x, c2, { {"label", "Relaxation optimized"}, {"linestyle", "-"}, {"color", "black"}, {"marker", "x"} });
	plot(x, c, { {"label", "Accurate"}, {"linestyle", "-"}, {"color", "red"}, {"marker", "None"} });
	title("Relaxation Optimize Experiment", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Iterations", { {"fontweight", "bold"} });
	ylabel("Average End Distance Cost", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageEndDistanceCost-Iterations.png");
	close();

	figure_size(1200, 800);
	plot(x, y1, { {"label", "Baseline"}, {"linestyle", "--"}, {"color", "black"}, {"marker", "o"} });
	plot(x, y2, { {"label", "Relaxation optimized"}, {"linestyle", "-"}, {"color", "black"}, {"marker", "x"} });
	title("Relaxation Optimize Experiment", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Iterations", { {"fontweight", "bold"} });
	ylabel("Average Distance Cost", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageDistanceCost-Iterations.png");
	close();

	figure_size(1200, 800);
	plot(x, t1, { {"label", "Baseline"}, {"linestyle", "--"}, {"color", "black"}, {"marker", "o"} });
	plot(x, t2, { {"label", "Relaxation optimized"}, {"linestyle", "-"}, {"color", "black"}, {"marker", "x"} });
	title("Relaxation Optimize Experiment", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Iterations", { {"fontweight", "bold"} });
	ylabel("Average Time Cost/s", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageTimeCost-Iterations.png");
	close();
}

void octree_optimize_experiment()
{
	init_robot_arm(3, 3);
	set_base_position(0, 3);
	set_end_position(0, 4.5, 0, 1.5);
	cout << "Basic settings: 3, 3; 0, 3; 0, 4.5, 0, 1.5" << endl;
	beam_config(5, WeightModel::THETA_LINEAR, 0, 0.7, 0.07, true);
	cout << "Beam Algorithm settings: 5, 0.07, theta linear model" << endl;
	cout << "Iterations: 3000; RRT Goal Bias Ratio: 0.3; Step Ratio: 1/3; neighbor range ratio: 1" << endl << endl << endl;

	ofstream file("RRT算法八叉树优化实验.csv");
	file << "实验配置：机械臂(3 3)，末端z坐标减小3" << endl;
	file << "Beam算法配置：束宽5，搜索步长0.07，θ-线性模型" << endl;
	file << "RRT算法配置：迭代次数3000，目标偏置0.3，步长比1/3，邻域半径比1，松弛优化" << endl << endl;
	file << "障碍物参数, (0.33 0.5), (0.33 1), (0.33 2), (0.67 0.5), (0.67 1), (0.67 2)" << endl;

	vector<string> x;
	vector<double> c1, c2, y1, y2, t1, t2;
	x.reserve(6);
	c1.reserve(6);
	c2.reserve(6);
	y1.reserve(6);
	y2.reserve(6);
	t1.reserve(6);
	t2.reserve(6);

	for (int o_opt = 0; o_opt <= 1; o_opt++)
	{
		vector<double>& c = o_opt ? c2 : c1;
		vector<double>& y = o_opt ? y2 : y1;
		vector<double>& t = o_opt ? t2 : t1;
		if (!o_opt)
		{
			cout << "Without Octree Optimization" << endl;
			file << "无八叉树优化, ";
		}
		else
		{
			cout << "With Octree Optimization" << endl;
			file << "八叉树优化, ";
		}
	    rrt_config(3000, 1.0 / 3, 0.3, 1, 0, true, o_opt, true, HeuristicFunction::NONE);

		set_obstacles(0.33, 0.5);
		cout << "Obstacle settings: 0.33, 0.5" << endl;
		x.push_back("(0.33, 0.5)");
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c.push_back(rrt_output.end_distance_cost);
		y.push_back(rrt_output.distance_cost);
		t.push_back(rrt_output.time_cost);
		file << rrt_output.time_cost << ", ";
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.33, 1);
		cout << "Obstacle settings: 0.33, 1" << endl;
		x.push_back("(0.33, 1)");
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c.push_back(rrt_output.end_distance_cost);
		y.push_back(rrt_output.distance_cost);
		t.push_back(rrt_output.time_cost);
		file << rrt_output.time_cost << ", ";
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.33, 2);
		cout << "Obstacle settings: 0.33, 2" << endl;
		x.push_back("(0.33, 2)");
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c.push_back(rrt_output.end_distance_cost);
		y.push_back(rrt_output.distance_cost);
		t.push_back(rrt_output.time_cost);
		file << rrt_output.time_cost << ", ";
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 0.5);
		cout << "Obstacle settings: 0.67, 0.5" << endl;
		x.push_back("(0.67, 0.5)");
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c.push_back(rrt_output.end_distance_cost);
		y.push_back(rrt_output.distance_cost);
		t.push_back(rrt_output.time_cost);
		file << rrt_output.time_cost << ", ";
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 1);
		cout << "Obstacle settings: 0.67, 1" << endl;
		x.push_back("(0.67, 1)");
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c.push_back(rrt_output.end_distance_cost);
		y.push_back(rrt_output.distance_cost);
		t.push_back(rrt_output.time_cost);
		file << rrt_output.time_cost << ", ";
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 2);
		cout << "Obstacle settings: 0.67, 2" << endl;
		x.push_back("(0.67, 2)");
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c.push_back(rrt_output.end_distance_cost);
		y.push_back(rrt_output.distance_cost);
		t.push_back(rrt_output.time_cost);
		file << rrt_output.time_cost << ", ";
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		cout << endl << "----------------------------------------" << endl << endl;
		file << endl;
	}
}

void heuristic_optimize_experiment()
{
	init_robot_arm(3, 3);
	set_base_position(0, 3);
	set_end_position(0, 4.5, 0, 1.5);
	cout << "Basic settings: 3, 3; 0, 3; 0, 4.5, 0, 1.5" << endl;
	set_obstacles(0.5, 1);
	cout << "Obstacle settings: 0.5, 1" << endl;
	beam_config(5, WeightModel::THETA_LINEAR, 0, 0.7, 0.07, true);
	cout << "Beam Algorithm settings: 5, 0.07, theta linear model" << endl;
	cout << "Iterations: 3000; RRT Goal Bias Ratio: 0.3; Step Ratio: 1/3; neighbor range ratio: 1" << endl << endl << endl;

	vector<double> x, c1, c2, c3, c4, y1, y2, y3, y4, t1, t2, t3, t4;
	x.reserve(10);
	c1.reserve(10);
	c2.reserve(10);
	c3.reserve(10);
	c4.reserve(10);
	y1.reserve(10);
	y2.reserve(10);
	y3.reserve(10);
	y4.reserve(10);
	t1.reserve(10);
	t2.reserve(10);
	t3.reserve(10);
	t4.reserve(10);

	rrt_config(3000, 1.0 / 3, 0.3, 1, 0, true, true, true, HeuristicFunction::NONE);
	cout << "Baseline:" << endl;
	for (int j = 0; j < 100; j++)
	{
		RRT_Output* p = new RRT_Output();
		*p = RRT_search();
		rrt_outputs.push_back(p);
		reset_rrt();
		reset_position();
	}
	rrt_output = print_rrt_average_cost(rrt_outputs);
	for(int i = 0; i < 10; i++)
	{
		c1.push_back(rrt_output.end_distance_cost);
		y1.push_back(rrt_output.distance_cost);
		t1.push_back(rrt_output.time_cost);
	}
	for (int j = 0; j < rrt_outputs.size(); j++)
		delete rrt_outputs[j];
	rrt_outputs.clear();

	for (double w = 0.1; w <= 1; w += 0.1)
	{
		x.push_back(w);
		cout << "Heuristic weight: " << w << endl << endl;

		rrt_config(3000, 1.0 / 3, 0.3, 1, w, true, true, true, HeuristicFunction::END);
		cout << "End Model" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c2.push_back(rrt_output.end_distance_cost);
		y2.push_back(rrt_output.distance_cost);
		t2.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		rrt_config(3000, 1.0 / 3, 0.3, 1, w, true, true, true, HeuristicFunction::BOTTLENECKPOINT);
		cout << "Bottleneck Point Model" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c3.push_back(rrt_output.end_distance_cost);
		y3.push_back(rrt_output.distance_cost);
		t3.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		rrt_config(3000, 1.0 / 3, 0.3, 1, w, true, true, true, HeuristicFunction::HYBRID);
		cout << "Hybrid Model" << endl;
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c4.push_back(rrt_output.end_distance_cost);
		y4.push_back(rrt_output.distance_cost);
		t4.push_back(rrt_output.time_cost);
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		cout << endl << "----------------------------------------" << endl << endl;
	}
	
	figure_size(1200, 800);
	plot(x, c1, { {"label", "Baseline"}, {"linestyle", "-"}, {"color", "red"}, { "marker", "None"}});
	plot(x, c2, { {"label", "End model"}, {"linestyle", "-"}, {"color", "yellow"}, {"marker", "o"} });
	plot(x, c3, { {"label", "Bottleneck point model"}, {"linestyle", "-"}, {"color", "blue"}, {"marker", "d"} });
	plot(x, c4, { {"label", "Hybrid model"}, {"linestyle", "-"}, {"color", "green"}, {"marker", "s"} });
	title("Heuristic Optimize Experiment", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Heuristic Weight", { {"fontweight", "bold"} });
	ylabel("Average End Distance Cost", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageEndDistanceCost-HeuristicWeight.png");
	close();

	figure_size(1200, 800);
	plot(x, y1, { {"label", "Baseline"}, {"linestyle", "-"}, {"color", "red"}, { "marker", "None"} });
	plot(x, y2, { {"label", "End model"}, {"linestyle", "-"}, {"color", "yellow"}, {"marker", "o"} });
	plot(x, y3, { {"label", "Bottleneck point model"}, {"linestyle", "-"}, {"color", "blue"}, {"marker", "d"} });
	plot(x, y4, { {"label", "Hybrid model"}, {"linestyle", "-"}, {"color", "green"}, {"marker", "s"} });
	title("Heuristic Optimize Experiment", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Heuristic Weight", { {"fontweight", "bold"} });
	ylabel("Average Distance Cost", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageDistanceCost-HeuristicWeight.png");
	close();

	figure_size(1200, 800);
	plot(x, t1, { {"label", "Baseline"}, {"linestyle", "-"}, {"color", "red"}, { "marker", "None"} });
	plot(x, t2, { {"label", "End model"}, {"linestyle", "-"}, {"color", "yellow"}, {"marker", "o"} });
	plot(x, t3, { {"label", "Bottleneck point model"}, {"linestyle", "-"}, {"color", "blue"}, {"marker", "d"} });
	plot(x, t4, { {"label", "Hybrid model"}, {"linestyle", "-"}, {"color", "green"}, {"marker", "s"} });
	title("Heuristic Optimize Experiment", { {"fontweight", "bold"}, {"fontsize", "16"} });
	xlabel("Heuristic weight", { {"fontweight", "bold"} });
	ylabel("Average Time Cost", { {"fontweight", "bold"} });
	legend();
	grid(true);
	save("AverageTimeCost-HeuristicWeight.png");
	close();
}

void optimization_validation()
{
	init_robot_arm(3, 3);
	set_base_position(0, 3);
	set_end_position(0, 4.5, 0, 1.5);
	cout << "Basic settings: 3, 3; 0, 3; 0, 4.5, 0, 1.5" << endl;
	beam_config(5, WeightModel::THETA_LINEAR, 0, 0.7, 0.07, true);
	cout << "Beam Algorithm settings: 5, 0.07, theta linear model" << endl;
	cout << "RRT Algorithm settings: 3000, 1/3, 0.3, 1, 1" << endl << endl << endl;

	ofstream file("RRT优化算法验证实验.csv");
	file << "实验配置：机械臂(3 3)，末端z坐标减小3" << endl;
	file << "Beam算法配置：束宽5，搜索步长0.07，θ-线性模型" << endl;
	file << "RRT算法配置：迭代次数3000，目标偏置0.3，步长比1/3，邻域半径比1，启发项权重1" << endl << endl;
	file << "障碍物参数, , (0.33 0.5), , , (0.33 1), , , (0.33 2), , , (0.67 0.5), , , (0.67 1), , , (0.67 2)" << endl;
	file << "指标, 平均终点代价, 平均总代价, 平均时间代价/s, 平均终点代价, 平均总代价, 平均时间代价/s, 平均终点代价, 平均总代价, 平均时间代价/s, 平均终点代价, 平均总代价, 平均时间代价/s, 平均终点代价, 平均总代价, 平均时间代价/s, 平均终点代价, 平均总代价, 平均时间代价/s" << endl;
	
	vector<string> x;
	vector<double> c1, c2, c3, y1, y2, y3, t1, t2, t3;
	x.reserve(6);
	c1.reserve(6);
	c2.reserve(6);
	c3.reserve(6);
	y1.reserve(6);
	y2.reserve(6);
	y3.reserve(6);
	t1.reserve(6);
	t2.reserve(6);
	t3.reserve(6);

	for (int i = 0; i < 3; i++)
	{
		vector<double>& c = i ? ((i == 1) ? c2 : c3) : c1;
		vector<double>& y = i ? ((i == 1) ? y2 : y3) : y1;
		vector<double>& t = i ? ((i == 1) ? t2 : t3) : t1;
		switch(i)
		{
		case 0:
			cout << "Baseline" << endl;
			file << "基线, ";
			rrt_config(3000, 1.0 / 3, 0.3, 0, 0, true, false, false, HeuristicFunction::NONE);
			break;
		case 1:
			cout << "Relaxation Optimize" << endl;
			file << "松弛优化, ";
			rrt_config(3000, 1.0 / 3, 0.3, 1, 0, true, false, true, HeuristicFunction::NONE);
			break;
		default:
			cout << "Relaxation, octree, heuristics Optimize" << endl;
			file << "松弛、八叉树、启发式优化, ";
			rrt_config(3000, 1.0 / 3, 0.3, 1, 1, true, true, true, HeuristicFunction::HYBRID);
			break;
		}

		set_obstacles(0.33, 0.5);
		cout << "Obstacle settings: 0.33, 0.5" << endl;
		x.push_back("(0.33, 0.5)");
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c.push_back(rrt_output.end_distance_cost);
		file << rrt_output.end_distance_cost << ", ";
		y.push_back(rrt_output.distance_cost);
		file << rrt_output.distance_cost << ", ";
		t.push_back(rrt_output.time_cost);
		file << rrt_output.time_cost << ", ";
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.33, 1);
		cout << "Obstacle settings: 0.33, 1" << endl;
		x.push_back("(0.33, 1)");
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c.push_back(rrt_output.end_distance_cost);
		file << rrt_output.end_distance_cost << ", ";
		y.push_back(rrt_output.distance_cost);
		file << rrt_output.distance_cost << ", ";
		t.push_back(rrt_output.time_cost);
		file << rrt_output.time_cost << ", ";
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.33, 2);
		cout << "Obstacle settings: 0.33, 2" << endl;
		x.push_back("(0.33, 2)");
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c.push_back(rrt_output.end_distance_cost);
		file << rrt_output.end_distance_cost << ", ";
		y.push_back(rrt_output.distance_cost);
		file << rrt_output.distance_cost << ", ";
		t.push_back(rrt_output.time_cost);
		file << rrt_output.time_cost << ", ";
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 0.5);
		cout << "Obstacle settings: 0.67, 0.5" << endl;
		x.push_back("(0.67, 0.5)");
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c.push_back(rrt_output.end_distance_cost);
		file << rrt_output.end_distance_cost << ", ";
		y.push_back(rrt_output.distance_cost);
		file << rrt_output.distance_cost << ", ";
		t.push_back(rrt_output.time_cost);
		file << rrt_output.time_cost << ", ";
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 1);
		cout << "Obstacle settings: 0.67, 1" << endl;
		x.push_back("(0.67, 1)");
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c.push_back(rrt_output.end_distance_cost);
		file << rrt_output.end_distance_cost << ", ";
		y.push_back(rrt_output.distance_cost);
		file << rrt_output.distance_cost << ", ";
		t.push_back(rrt_output.time_cost);
		file << rrt_output.time_cost << ", ";
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		set_obstacles(0.67, 2);
		cout << "Obstacle settings: 0.67, 2" << endl;
		x.push_back("(0.67, 2)");
		for (int j = 0; j < 100; j++)
		{
			RRT_Output* p = new RRT_Output();
			*p = RRT_search();
			rrt_outputs.push_back(p);
			reset_rrt();
			reset_position();
		}
		rrt_output = print_rrt_average_cost(rrt_outputs);
		c.push_back(rrt_output.end_distance_cost);
		file << rrt_output.end_distance_cost << ", ";
		y.push_back(rrt_output.distance_cost);
		file << rrt_output.distance_cost << ", ";
		t.push_back(rrt_output.time_cost);
		file << rrt_output.time_cost << ", ";
		for (int j = 0; j < rrt_outputs.size(); j++)
			delete rrt_outputs[j];
		rrt_outputs.clear();

		cout << endl << "----------------------------------------" << endl << endl;
		file << endl;
	}
}

/*
================================
Comprehensive Experiment
================================
 */

void final_optimization_assessment()
{
	cout << "Comprehensive Experiment" << endl;
	ofstream file("综合实验.csv");
	init_robot_arm(5, 5);
	cout << "Robot arm settings: 5, 5" << endl;
	file << "机械臂配置：第一段长度5，第二段长度5" << endl;
	set_obstacles(0.5, 1.5);
	cout << "Obstacle settings: 0.5, 1.5" << endl;
	file << "障碍物配置：宽度占比0.5，高度1.5" << endl << endl << endl;

	vector<string> x;
	vector<double> d1, d2, t1, t2;
	x.reserve(3);
	d1.reserve(3);
	d2.reserve(3);
	t1.reserve(3);
	t2.reserve(3);

	for(int optimize = 0; optimize <= 1; optimize++)
	{
		vector<double>& d = optimize ? d2 : d1;
		vector<double>& t = optimize ? t2 : t1;
		if (!optimize)
		{
			cout << "Baseline" << endl;
			file << "基线" << endl;
			beam_config(5, WeightModel::NONE, 0, 0.7, 0.07, false);
			cout << "Beam Algorithm settings: 5, 0.07" << endl;
			file << "Beam原始算法配置：束宽5，搜索步长0.07" << endl;
			rrt_config(3000, 1.0 / 3, 0.3, 0, 0, true, false, false, HeuristicFunction::NONE);
			cout << "RRT Algorithm settings: 3000, 1/3, 0.3" << endl << endl;
			file << "RRT原始算法配置：迭代次数上限3000，目标偏置0.3，步长比1/3" << endl << endl;
		}
		else
		{
			cout << "Optimized" << endl;
			file << "优化算法" << endl;
			beam_config(5, WeightModel::THETA_LINEAR, 0, 0.7, 0.07, true);
			cout << "Optimized Beam Algorithm settings: 5, 0.07, theta linear model" << endl;
			file << "Beam优化算法配置：束宽5，搜索步长0.07，θ-线性模型" << endl;
			rrt_config(3000, 1.0 / 3, 0.3, 1, 1, true, true, true, HeuristicFunction::HYBRID);
			cout << "Optimized RRT Algorithm settings: 3000, 1/3, 0.3, 1, 1, hybrid heuristic function" << endl << endl;
			file << "RRT优化算法配置：迭代次数上限3000，目标偏置0.3，步长比1/3，邻域半径比1，启发项权重1，混合式启发函数" << endl << endl;
		}
		file << "组别, 平均代价, 平均耗时(s)" << endl;

		x.push_back("Horizontal");
		cout << "Horizontal" << endl << endl;
		file << "水平组, ";
		for(int i = 0; i < 100; i++)
		{
			Beam_Output* p = new Beam_Output({ true, 0, 0 });
			set_base_position(0, 5);

			set_end_position(0, 3, 8, 3);
			beam_output = beam_search();
			if (beam_output.success)
			{
				p->distance_cost += beam_output.distance_cost;
				p->time_cost += beam_output.time_cost;
			}
			else
				p->success = false;
			reset_beam();
			update_base_position();

			set_end_position(8, 3, 8, 5);
			rrt_output = RRT_search();
			if (rrt_output.success)
			{
				p->distance_cost += rrt_output.distance_cost;
				p->time_cost += rrt_output.time_cost;
			}
			else
				p->success = false;
			reset_rrt();
			update_base_position();

			set_end_position(8, 5, 0, 5);
			beam_output = beam_search();
			if (beam_output.success)
			{
				p->distance_cost += beam_output.distance_cost;
				p->time_cost += beam_output.time_cost;
			}
			else
				p->success = false;
			reset_beam();
			update_base_position();

			set_end_position(0, 5, 0, 7);
			rrt_output = RRT_search();
			if (rrt_output.success)
			{
				p->distance_cost += rrt_output.distance_cost;
				p->time_cost += rrt_output.time_cost;
			}
			else
				p->success = false;
			reset_rrt();
			update_base_position();

			set_end_position(0, 7, 8, 7);
			beam_output = beam_search();
			if (beam_output.success)
			{
				p->distance_cost += beam_output.distance_cost;
				p->time_cost += beam_output.time_cost;
			}
			else
				p->success = false;
			reset_beam();
			update_base_position();

			total_outputs.push_back(p);
		}
		total_output = print_average_total_cost(total_outputs);
		if(total_output.success)
		{
			d.push_back(total_output.distance_cost);
			t.push_back(total_output.time_cost);
			file << total_output.distance_cost << ", " << total_output.time_cost << endl;
		}
		else
		{
			d.push_back(0);
			t.push_back(0);
			file << "失败" << endl;
		}
		for (int i = 0; i < total_outputs.size(); i++)
			delete total_outputs[i];
		total_outputs.clear();

		x.push_back("Inclined");
		cout << "Inclined" << endl << endl;
		file << "倾斜组, ";
		for (int i = 0; i < 100; i++)
		{
			Beam_Output* p = new Beam_Output({ true, 0, 0 });
			set_base_position(0, 5);

			set_end_position(0, 1, 6, 7);
			beam_output = beam_search();
			if (beam_output.success)
			{
				p->distance_cost += beam_output.distance_cost;
				p->time_cost += beam_output.time_cost;
			}
			else
				p->success = false;
			reset_beam();
			update_base_position();

			set_end_position(6, 7, 5, 8);
			rrt_output = RRT_search();
			if (rrt_output.success)
			{
				p->distance_cost += rrt_output.distance_cost;
				p->time_cost += rrt_output.time_cost;
			}
			else
				p->success = false;
			reset_rrt();
			update_base_position();

			set_end_position(5, 8, -1, 2);
			beam_output = beam_search();
			if (beam_output.success)
			{
				p->distance_cost += beam_output.distance_cost;
				p->time_cost += beam_output.time_cost;
			}
			else
				p->success = false;
			reset_beam();
			update_base_position();

			set_end_position(-1, 2, -2, 3);
			rrt_output = RRT_search();
			if (rrt_output.success)
			{
				p->distance_cost += rrt_output.distance_cost;
				p->time_cost += rrt_output.time_cost;
			}
			else
				p->success = false;
			reset_rrt();
			update_base_position();

			set_end_position(-2, 3, 4, 9);
			beam_output = beam_search();
			if (beam_output.success)
			{
				p->distance_cost += beam_output.distance_cost;
				p->time_cost += beam_output.time_cost;
			}
			else
				p->success = false;
			reset_beam();
			update_base_position();

			total_outputs.push_back(p);
		}
		total_output = print_average_total_cost(total_outputs);
		if (total_output.success)
		{
			d.push_back(total_output.distance_cost);
			t.push_back(total_output.time_cost);
			file << total_output.distance_cost << ", " << total_output.time_cost << endl;
		}
		else
		{
			d.push_back(0);
			t.push_back(0);
			file << "失败" << endl;
		}
		for (int i = 0; i < total_outputs.size(); i++)
			delete total_outputs[i];
		total_outputs.clear();

		x.push_back("Vertical");
		cout << "Vertical" << endl << endl;
		file << "垂直组, ";
		for (int i = 0; i < 100; i++)
		{
			Beam_Output* p = new Beam_Output({ true, 0, 0 });
			set_base_position(0, 5);

			set_end_position(0, 1, 0, 9);
			beam_output = beam_search();
			if (beam_output.success)
			{
				p->distance_cost += beam_output.distance_cost;
				p->time_cost += beam_output.time_cost;
			}
			else
				p->success = false;
			reset_beam();
			update_base_position();

			set_end_position(0, 9, 2, 9);
			rrt_output = RRT_search();
			if (rrt_output.success)
			{
				p->distance_cost += rrt_output.distance_cost;
				p->time_cost += rrt_output.time_cost;
			}
			else
				p->success = false;
			reset_rrt();
			update_base_position();

			set_end_position(2, 9, 2, 1);
			beam_output = beam_search();
			if (beam_output.success)
			{
				p->distance_cost += beam_output.distance_cost;
				p->time_cost += beam_output.time_cost;
			}
			else
				p->success = false;
			reset_beam();
			update_base_position();

			set_end_position(2, 1, 4, 1);
			rrt_output = RRT_search();
			if (rrt_output.success)
			{
				p->distance_cost += rrt_output.distance_cost;
				p->time_cost += rrt_output.time_cost;
			}
			else
				p->success = false;
			reset_rrt();
			update_base_position();

			set_end_position(4, 1, 4, 9);
			beam_output = beam_search();
			if (beam_output.success)
			{
				p->distance_cost += beam_output.distance_cost;
				p->time_cost += beam_output.time_cost;
			}
			else
				p->success = false;
			reset_beam();
			update_base_position();

			total_outputs.push_back(p);
		}
		total_output = print_average_total_cost(total_outputs);
		if (total_output.success)
		{
			d.push_back(total_output.distance_cost);
			t.push_back(total_output.time_cost);
			file << total_output.distance_cost << ", " << total_output.time_cost << endl;
		}
		else
		{
			d.push_back(0);
			t.push_back(0);
			file << "失败" << endl;
		}
		for (int i = 0; i < total_outputs.size(); i++)
			delete total_outputs[i];
		total_outputs.clear();


		cout << endl << "----------------------------------------" << endl << endl;
		file << endl;
	}
}