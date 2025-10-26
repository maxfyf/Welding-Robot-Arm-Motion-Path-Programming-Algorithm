#include "global.h"
#include <cmath>
using namespace std;

double l1;    //基座到关节的距离
double l2;    //关节到末端的距离
double x_b1, y_b1;    //基座起点坐标
double x_b2, y_b2;	 //基座终点坐标
double x_e1, z_e1;    //末端起点坐标
double x_e2, z_e2;    //末端终点坐标

//备份初始化坐标
double _xb1, _yb1;
double _xe1, _ze1;
double _xe2, _ze2;


void init_robot_arm(double L1, double L2)
{
	l1 = L1;
	l2 = L2;
}

void set_base_position(double xb, double yb)
{
	_xb1 = x_b1 = xb;
	_yb1 = y_b1 = yb;
}

void update_base_position()
{
	x_b1 = x_b2;
	y_b1 = y_b2;
}

void reset_base_position()
{
	x_b1 = _xb1;
	y_b1 = _yb1;
}

void set_end_position(double xe1, double ze1, double xe2, double ze2)
{

	_xe1 = x_e1 = xe1;
	_ze1 = z_e1 = ze1;
	_xe2 = x_e2 = xe2;
	_ze2 = z_e2 = ze2;
}

void update_end_position(double xe, double ze)
{
	x_e1 = x_e2;
	z_e1 = z_e2;
	x_e2 = xe;
	z_e2 = ze;
}

void reset_end_position()
{
	x_e1 = _xe1;
	z_e1 = _ze1;
	x_e2 = _xe2;
	z_e2 = _ze2;
}

void reset_position()
{
	reset_base_position();
	reset_end_position();
}

bool check()
{
	if (y_b1 < 0)
		return false;
	if (dist3(x_b1, y_b1, 0, x_e1, 0, z_e1) > l1 + l2)
		return false;
	return true;
}

double dist2(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double dist3(double x1, double y1, double z1, double x2, double y2, double z2)
{
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1));
}

double max(double a, double b)
{
	return a > b ? a : b;
}

bool calculate_joint_position(double x0, double y0, double _x, double _y, double _z, double& x, double& y, double& z)
{
	y0 -= _y;
	double s = sqrt((x0 - _x) * (x0 - _x) + y0 * y0);    //焊点在基座平面上的投影到基座的距离

	//设关节相对于焊点的高度为h，有关于h的一元二次方程，系数a、b、c
	double a = 4 * s * s + 4 * _z * _z;
	double b = 4 * _z * (s * s + _z * _z - l1 * l1 + l2 * l2);
	double c = (s * s + _z * _z - l1 * l1 + l2 * l2) * (s * s + _z * _z - l1 * l1 + l2 * l2) - 4 * l2 * l2 * s * s;
	if (b * b < 4 * a * c) return false;
	double h1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
	double h2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
	double h = h1 > h2 ? h1 : h2;
	z = _z + h;
	double s0 = sqrt(l2 * l2 - h * h);    //关节到焊点在基座平面上投影的距离
	x = _x + s0 / s * (x0 - _x);    //关节x坐标
	y = s0 / s * y0 + _y;    //关节y坐标
	return true;
}