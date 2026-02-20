#ifndef BEAM_H
#define BEAM_H

#include "global.h"
#include <vector>
#define RATIO 0.6972    //平分半圆面积且平行于半圆直径的弦到半圆直径的距离与半径的比值 
#define ITER_RATIO 10    //搜索步数与最小步数的比值
using namespace std;

typedef struct
{
	bool success;
	double distance_cost;
	double time_cost;
}Beam_Output;

enum class WeightModel
{
	NONE,
	Z_LINEAR,
	Z_EXPONENTIAL,
	THETA_LINEAR,
	THETA_LOGARITHMIC
};

extern int K;    //束宽
extern double BEAM_W;    //启发项权重 
extern double range;    //基座在x或y方向位移搜索上下限的绝对值
extern double beam_step;    //搜索步长
extern bool beam_optimize;	//是否优化

extern vector<Base> beam_base_path;    //基座路径
extern vector<Joint> beam_joint_path;    //关节路径

void beam_config(int k, WeightModel wm, double w, double r, double s, bool opt);    //配置Beam算法参数
void reset_beam();    //清空Beam算法除了参数以外的所有内存
Beam_Output _beam_search(double x1, double y1, double z1, double x2, double y2, double z2);    //Beam Search算法(三维末端)
Beam_Output beam_search();    //Beam Search算法（二维末端）

#endif