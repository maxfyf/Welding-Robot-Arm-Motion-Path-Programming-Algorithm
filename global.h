#ifndef GLOBAL_H
#define GLOBAL_H
#include <limits>
#define MAX_DBL std::numeric_limits<double>::max()
#define PI 3.1415926

using namespace std;
					 
extern double l1;    //基座到关节的距离
extern double l2;    //关节到末端的距离
extern double lr;    //障碍物在始末位置连线上的宽度占比
extern double h0;    //障碍物高度
extern double x_b1, y_b1;    //基座起点坐标
extern double x_b2, y_b2;	 //基座终点坐标
extern double x_e1, z_e1;    //末端起点坐标
extern double x_e2, z_e2;    //末端终点坐标

typedef struct
{
	double x;
	double y;
}Base;

typedef struct
{
	double x;
	double y;
	double z;
}Joint;

typedef struct
{
	double x;
	double y;
	double z;
}End;

void init_robot_arm(double L1, double L2);    //初始化机械臂参数
void set_obstacles(double r, double h);    //设置障碍物参数
void set_base_position(double xb, double yb);    //设置基座起点位置
void update_base_position();    //更新基座起点位置为上一次搜索得到的基座终点位置
void reset_base_position();    //重置基座起点位置为初始位置
void set_end_position(double xe1, double ze1, double xe2, double ze2);    //设置末端起点与终点位置
void update_end_position(double xe, double ze);    //更新末端起点位置为上一次搜索得到的末端终点位置,并设置新的末端终点位置
void reset_end_position();    //重置末端起点与终点位置为初始位置
void reset_position();    //重置机械臂位置为初始位置
bool check();    //检查初始化参数是否合法
void set_random_case();    //设置随机测试用例
double dist2(double x1, double y1, double x2, double y2);    //计算二维距离
double dist3(double x1, double y1, double z1, double x2, double y2, double z2);    //计算三维距离
double max(double a, double b);    //返回较大值
bool calculate_joint_position(double x0, double y0, double _x, double _y, double _z, double& x, double& y, double& z);    //根据基座位置与末端位置计算关节位置
double calculate_angle(double dist);    //计算机械臂间的夹角
void projection(double x, double y, double& x_h, double& y_h, double& r);    //计算点到线段的投影点
void linear_equations(double a1, double b1, double c1, double a2, double b2, double c2, bool& solvable, double& x, double& y);    //解二元一次方程组
bool hit(double xa, double ya, double za, double xb, double yb, double zb);    //判断线段是否与障碍物相交
double dist_point_cuboid(double x, double y, double z, double xc, double yc, double zc, double xw, double yw, double zw);    //计算点到长方体的距离

#endif