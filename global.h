#ifndef GLOBAL_H
#define GLOBAL_H
#define PI 3.1415926
#define ITER_RATIO 10    //迭代次数与最少次数的比值

using namespace std;
					 
extern double l1;    //基座到关节的距离
extern double l2;    //关节到末端的距离
extern double x_b1, y_b1;    //基座起点坐标
extern double x_b2, y_b2;	 //基座终点坐标
extern double x_e1, z_e1;    //末端起点坐标
extern double x_e2, z_e2;    //末端终点坐标

typedef struct
{
	double x, y;
}Base;

typedef struct
{
	double x, y, z;
}Joint;

typedef struct
{
	bool success;
	double distance_cost;
	double time_cost;
}Output;

void set_robot_arm(double L1, double L2, double xb, double yb, double xe1, double ze1, double xe2, double ze2);    //初始化机械臂参数
bool check();    //检查初始化参数是否合法
double dist2(double x1, double y1, double x2, double y2);    //计算二维距离
double dist3(double x1, double y1, double z1, double x2, double y2, double z2);    //计算三维距离
double max(double a, double b);    //返回较大值
bool calculate_joint_position(double x0, double y0, double _x, double _z, double& x, double& y, double& z);    //根据基座位置与末端位置计算关节位置

#endif