#ifndef TESTS_H
#define TESTS_H
#include "Beam.h"
#include "RRT.h"
#include "matplotlibcpp.h"
using namespace matplotlibcpp;

extern Beam_Output beam_output;
extern RRT_Output rrt_output;
extern vector<RRT_Output*> rrt_outputs;

void Beam_test();
void RRT_test();

void adjust_beam_width();
void adjust_beam_step();
void primary_Beam_experiment();
void Beam_experiment();
void adjust_beam_weight_model();

void RRT_goal_bias_experiment();
void adjust_rrt_step_ratio();
void adjust_rrt_neighbor_range_ratio();
void relaxation_optimize_experiment();
void octree_optimize_experiment();

#endif