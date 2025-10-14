#ifndef BEAM_H
#define BEAM_H

#include "global.h"
#include <vector>
using namespace std;

extern int K;    //束宽
extern double W1, W2;    //启发函数中机械臂夹角与基座偏移角的权重 
extern double range;    //基座在x或y方向位移搜索上下限的绝对值
extern double beam_step;    //搜索步长
extern bool beam_optimize;	//是否优化

extern vector<Base> beam_base_path;    //基座路径
extern vector<Joint> beam_joint_path;    //关节路径

Output beam_search(int k, double w1, double w2, double r, double s, bool opt);    //Beam Search算法

#endif