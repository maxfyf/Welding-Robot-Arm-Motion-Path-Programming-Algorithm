#ifndef LOG_H
#define LOG_H

#include "Beam.h"
#include "RRT.h"

void print_beam_distance_cost(Beam_Output& beam_output);
void print_beam_base_path();
void print_beam_joint_path();
void print_beam_path();
void print_beam_time_cost(Beam_Output& beam_output);
void demonstrate_beam_results(Beam_Output& beam_output);
void print_beam_cost(Beam_Output& beam_output);

void print_rrt_end_distance_cost(RRT_Output& rrt_output);
void print_rrt_average_end_distance_cost(vector<RRT_Output*>& rrt_outputs);
void print_rrt_distance_cost(RRT_Output& rrt_output);
void print_rrt_average_distance_cost(vector<RRT_Output*>& rrt_outputs);
void print_rrt_base_path();
void print_rrt_joint_path();
void print_rrt_end_path();
void print_rrt_path();
void print_rrt_time_cost(RRT_Output& rrt_output);
void print_rrt_average_time_cost(vector<RRT_Output*>& rrt_outputs);
void demonstrate_rrt_results(RRT_Output& rrt_output);
void print_rrt_cost(RRT_Output& rrt_output);
void print_rrt_average_cost(vector<RRT_Output*>& rrt_outputs);

#endif