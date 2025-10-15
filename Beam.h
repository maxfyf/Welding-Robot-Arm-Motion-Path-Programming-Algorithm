#ifndef BEAM_H
#define BEAM_H

#include "global.h"
#include <vector>
#define RATIO 0.6972    //平分半圆面积且平行于半圆直径的弦到半圆直径的距离与半径的比值 
using namespace std;

extern int K;    //束宽
extern double BEAM_W;    //启发项权重 
extern double range;    //基座在x或y方向位移搜索上下限的绝对值
extern double beam_step;    //搜索步长
extern bool beam_optimize;	//是否优化

extern vector<Base> beam_base_path;    //基座路径
extern vector<Joint> beam_joint_path;    //关节路径

Output beam_search(int k, double w, double r, double s, bool opt);    //Beam Search算法

#endif