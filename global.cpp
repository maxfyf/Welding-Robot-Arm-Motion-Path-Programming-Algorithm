#include "global.h"
#include <cmath>
#include <random>

double l1;    //基座到关节的距离
double l2;    //关节到末端的距离
double lr;    //障碍物在始末位置连线上的宽度占比
double h0;    //障碍物高度
double x_b1, y_b1;    //基座起点坐标
double x_b2, y_b2;	  //基座终点坐标
double x_e1, z_e1;    //末端起点坐标
double x_e2, z_e2;    //末端终点坐标

//备份初始化坐标
double _xb1, _yb1;
double _xe1, _ze1;
double _xe2, _ze2;

//正态分布随机数生成器
random_device rd;
mt19937 gen(rd());
normal_distribution<double> distribution(3.0, 1.0);

void init_robot_arm(double L1, double L2)
{
	l1 = L1 > 1 ? L1 : 1;
	l2 = L2 > 1 ? L2 : 1;
}

void set_obstacles(double r, double h)
{
	if (r >= 0.9) lr = 0.9;
	else if(r <= 0) lr = 0;
	else lr = r;
	h0 = h > 0 ? h : 0;
}

void set_base_position(double xb, double yb)
{
	_xb1 = x_b1 = xb;
	_yb1 = y_b1 = yb > 0 ? yb : 0;
}

void update_base_position()
{
	x_b1 = x_b2;
	y_b1 = y_b2 > 0 ? y_b2 : 0;
}

void reset_base_position()
{
	x_b1 = _xb1;
	y_b1 = _yb1;
}

void set_end_position(double xe1, double ze1, double xe2, double ze2)
{

	_xe1 = x_e1 = xe1;
	_ze1 = z_e1 = ze1 > 0 ? ze1 : 0;
	_xe2 = x_e2 = xe2;
	_ze2 = z_e2 = ze2 > 0 ? ze2 : 0;
}

void update_end_position(double xe, double ze)
{
	x_e1 = x_e2;
	z_e1 = z_e2;
	x_e2 = xe;
	z_e2 = ze > 0 ? ze : 0;
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
	if (y_b1 < 0 || z_e1 < 0)
		return false;
	if (dist3(x_b1, y_b1, 0, x_e1, 0, z_e1) > l1 + l2 - 0.5)
		return false;
	return true;
}

void set_random_case()
{
	do
	{
		_xb1 = x_b1 = 0;
		_yb1 = y_b1 = distribution(gen);
		_xe1 = x_e1 = 0;
		_ze1 = z_e1 = distribution(gen);
		_xe2 = x_e2 = 10;
		_ze2 = z_e2 = distribution(gen);
	} while (!check());
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

double calculate_angle(double dist)
{
	double cosine = (l1 * l1 + l2 * l2 - dist * dist) / (2 * l1 * l2);
	if(cosine >= 0) return acos(cosine);
	else return PI - acos(-cosine);
}

void projection(double x, double z, double &x_h, double &z_h, double &r)
{
	double dot = (x - x_e1) * (x_e2 - x_e1) + (z - z_e1) * (z_e2 - z_e1);
	x_h = x_e1 + dot * (x_e2 - x_e1) / ((x_e2 - x_e1) * (x_e2 - x_e1) + (z_e2 - z_e1) * (z_e2 - z_e1));
	z_h = z_e1 + dot * (z_e2 - z_e1) / ((x_e2 - x_e1) * (x_e2 - x_e1) + (z_e2 - z_e1) * (z_e2 - z_e1));
	r = (x_h - x_e1) / (x_e2 - x_e1);
}

void linear_equations(double a1, double b1, double c1, double a2, double b2, double c2, bool& solvable, double& x, double& y)
{
	double det = a1 * b2 - a2 * b1;
	if (det == 0)
	{
		solvable = false;
		return;
	}
	solvable = true;
	x = (c1 * b2 - c2 * b1) / det;
	y = (a1 * c2 - a2 * c1) / det;
}

bool hit(double xa, double ya, double za, double xb, double yb, double zb)
{
	if (ya < 0 || yb < 0 || za < 0 || zb < 0) return false;
	double a0, b0, c1, c2, a, b, c;
	a0 = x_e1 - x_e2;
	b0 = z_e1 - z_e2;
	c1 = a0 * ((1 - lr) * x_e2 + (1 + lr) * x_e1) / 2 + b0 * ((1 - lr) * z_e2 + (1 + lr) * z_e1) / 2;
	c2 = a0 * ((1 + lr) * x_e2 + (1 - lr) * x_e1) / 2 + b0 * ((1 + lr) * z_e2 + (1 - lr) * z_e1) / 2;
	a = zb - za;
	b = xb - xa;
	c = a * xa + b * za;
	bool solvable;
	double xh1, zh1, xh2, zh2;
	linear_equations(a0, b0, c1, a, b, c, solvable, xh1, zh1);
	linear_equations(a0, b0, c2, a, b, c, solvable, xh2, zh2);
	
	double temp, h_min;
	if (solvable)
	{
		if (xa > xb)
		{
			temp = xa;
			xa = xb;
			xb = temp;
			temp = ya;
			ya = yb;
			yb = temp;
			temp = za;
			za = zb;
			zb = temp;
		}
		if (xh1 > xh2)
		{
			temp = xh1;
			xh1 = xh2;
			xh2 = temp;
			temp = zh1;
			zh1 = zh2;
			zh2 = temp;
		}
		if (xh1 < xa) xh1 = xa;
		if (xh2 > xb) xh2 = xb;
		if (xh2 < xh1) return false;
		if (ya < yb) h_min = (xh1 - xa) / (xb - xa) * (yb - ya) + ya;
		else h_min = (xh2 - xa) / (xb - xa) * (yb - ya) + ya;
		return (h_min <= h0);
	}
	else
	{
		double r;
		projection(xa, za, xh1, zh1, r);
		if (r < (1 - lr) / 2 || r >(1 + lr) / 2) return false;
		else return (ya <= h0 || yb <= h0);
	}
}