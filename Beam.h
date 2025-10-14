#ifndef BEAM_H
#define BEAM_H

#include "global.h"
#include <vector>
using namespace std;

extern int K;    //束宽
extern double W1, W2;    //启发函数中机械臂夹角与基座偏移角的权重 
extern double lbound, ubound;    //基座在x或y方向位移搜索下限与上限
extern double step;    //搜索步长
extern bool optimize;	//是否优化

extern vector<Base> base_path;    //基座路径
extern vector<Joint> joint_path;    //关节路径

typedef struct
{
	bool success;
	double distance_cost;
	double time_cost;
}BeamOutput;

BeamOutput beam_search(int k, double w1, double w2, double l, double u, double s, bool opt);    //Beam Search算法

#endif