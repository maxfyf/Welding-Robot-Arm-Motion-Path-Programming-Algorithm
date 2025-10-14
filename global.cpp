#include "global.h"
#include <cmath>
#define PI 3.1415926
using namespace std;

double l1;    //基座到关节的距离
double l2;    //关节到末端的距离
double x_b1, y_b1;    //基座起点坐标
double x_b2, y_b2;	 //基座终点坐标
double x_e1, z_e1;    //末端起点坐标
double x_e2, z_e2;    //末端终点坐标

void init_robot_arm(double L1, double L2, double xb, double yb, double xe1, double ze1, double xe2, double ze2)
{
	l1 = L1;
	l2 = L2;
	x_b1 = xb;
	y_b1 = yb;
	x_e1 = xe1;
	z_e1 = ze1;
	x_e2 = xe2;
	z_e2 = ze2;
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

bool calculate_joint_position(double x0, double y0, double _x, double _z, double& x, double& y, double& z)
{
	double s = sqrt((x0 - _x) * (x0 - _x) + y0 * y0);    //焊点在基座平面上的投影到基座的距离

	//设关节相对于焊点的高度为h，有关于h的一元二次方程，系数a、b、c
	double a = 4 * s * s + 4 * _z * _z;
	double b = 4 * _z * (s * s + _z * _z - l1 * l1 + l2 * l2);
	double c = (s * s + _z * _z - l1 * l1 + l2 * l2) * (s * s + _z * _z - l1 * l1 + l2 * l2) - 4 * l2 * l2 * s * s;
	if (b* b < 4 * a * c) return false;
	double h1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
	double h2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
	double h = h1 > h2 ? h1 : h2;
	z = _z + h;
	double s0 = sqrt(l2 * l2 - h * h);    //关节到焊点在基座平面上投影的距离
	x = _x + s0 / s * (x0 - _x);    //关节x坐标
	y = s0 / s * y0;    //关节y坐标
	return true;
}

double calculate_included_angle(double x0, double y0, double x, double y, double z, double _x, double _z)
{
	double v1_x = x0 - x;
	double v1_y = y0 - y;
	double v1_z = -z;
	double v2_x = _x - x;
	double v2_y = -y;
	double v2_z = _z - z;
	double dot_product = v1_x * v2_x + v1_y * v2_y + v1_z * v2_z;
	double magnitude_v1 = sqrt(v1_x * v1_x + v1_y * v1_y + v1_z * v1_z);
	double magnitude_v2 = sqrt(v2_x * v2_x + v2_y * v2_y + v2_z * v2_z);
	double cosine_angle = dot_product / (magnitude_v1 * magnitude_v2);
	if (cosine_angle >= 0) return acos(cosine_angle);
	else return PI - acos(-cosine_angle);
}

double calculate_bias_angle(double x0, double y0, double _x)
{
	double dx = abs(_x - x0);
	double dy = abs(y0);
	return atan2(dx, dy);
}