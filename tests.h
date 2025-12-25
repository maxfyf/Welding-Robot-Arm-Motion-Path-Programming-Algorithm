#ifndef TESTS_H
#define TESTS_H
#include "Beam.h"
#include "RRT.h"
#include "matplotlibcpp.h"
using namespace matplotlibcpp;

extern Beam_Output beam_output;
extern RRT_Output rrt_output;
extern vector<RRT_Output*> rrt_outputs;

void adjust_beam_width();
void adjust_beam_step();
void adjust_rrt_step_and_neighbor_ratio();
void primary_Beam_experiment();
void Beam_experiment();
void adjust_beam_weight_model();

#endif