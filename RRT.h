#ifndef RRT_H
#define RRT_H

#include "global.h"
#include <vector>
using namespace std;

typedef struct
{
	bool success;
	double end_distance_cost;
	double distance_cost;
	double time_cost;
}RRT_Output;

extern double rrt_step_ratio;    //搜索步长
extern double goal_bias;    //目标偏置频率
extern double neighbor_range_ratio;    //邻域半径与搜索步长的比值
extern double RRT_W;    //启发项权重
extern bool goal_bias_optimize;	   //是否目标偏置优化
extern bool relaxation_optimize;    //是否松弛优化
extern bool heuristic_optimize;    //是否启发式优化

extern vector<Base> rrt_base_path;    //基座路径
extern vector<Joint> rrt_joint_path;    //关节路径
extern vector<End> rrt_end_path;    //末端路径

void rrt_config(int it, double sr, double gb, double nrr, double w, bool gb_opt, bool r_opt, bool h_opt);    //配置RRT算法参数
void reset_rrt();    //清空RRT算法除了参数以外的所有内存
RRT_Output RRT_search();    //RRT算法

#endif