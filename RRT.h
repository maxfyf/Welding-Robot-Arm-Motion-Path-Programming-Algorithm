#ifndef RRT_H
#define RRT_H

#include "global.h"
#include <vector>
using namespace std;

extern double rrt_step_ratio;    //搜索步长
extern double goal_bias;    //目标偏置频率
extern double neighbor_range_ratio;    //邻域半径与搜索步长的比值
extern double std_ratio;    //随机点和终点间距正态分布的标准差与起点和终点间距的比值
extern double RRT_W;    //启发项权重
extern bool goal_bias_optimize;	   //是否目标偏置优化
extern bool relaxation_optimize;    //是否松弛优化
extern bool heuristic_optimize;    //是否启发式优化

extern vector<Base> rrt_base_path;    //基座路径
extern vector<Joint> rrt_joint_path;    //关节路径
extern vector<End> rrt_end_path;    //末端路径

#endif