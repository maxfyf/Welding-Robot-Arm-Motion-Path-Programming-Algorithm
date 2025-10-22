#include "global.h"
#include "rrt.h"
#include "beam.h"
#include <new>
#include <vector>
#include <map>
#include <unordered_set>
#include <cmath>
#include <random>
#include <chrono>
using namespace std;

double rrt_step_ratio;    //搜索步长与起点和终点间距的比值
double goal_bias;    //目标偏置频率
double neighbor_range_ratio;    //邻域半径与搜索步长的比值
double std_ratio;    //随机点和终点间距正态分布的标准差与起点和终点间距的比值
double RRT_W;    //启发项权重
bool goal_bias_optimize;	   //是否目标偏置优化
bool relaxation_optimize;    //是否松弛优化
bool heuristic_optimize;    //是否启发式优化

vector<Base> rrt_base_path;    //基座路径
vector<Joint> rrt_joint_path;    //关节路径
vector<End> rrt_end_path;    //末端路径

double std_dev;    //随机点正态分布标准差
int rrt_n;    //迭代次数
double step;    //搜索树生长步长
double neighbor_range;    //邻域半径

typedef struct Node
{
	double x;
	double y;
	double z;
	double g;    //代价
	double f;    //代价+启发项
	struct Node* parent;
}Node;    //搜索树上结点
static vector<Node*> tree;    //搜索树
static multimap<Node*, Node*> adjlist;    //邻接表
static multimap<double, Node*> openlist;    //开放列表
static unordered_set<Node*> closedlist;    //封闭列表

void rrt_config(double sr, double gb, double nrr, double stdr, double w, bool gb_opt, bool r_opt, bool h_opt)
{
	rrt_step_ratio = sr;
	goal_bias = gb;
	neighbor_range_ratio = nrr;
	std_ratio = stdr;
	RRT_W = w;
	goal_bias_optimize = gb_opt;
	relaxation_optimize = r_opt;
	heuristic_optimize = h_opt && r_opt;
}

void reset_rrt()
{
	rrt_base_path.clear();
	rrt_joint_path.clear();
	rrt_end_path.clear();
	for (int i = 0; i < tree.size(); i++)
		delete tree[i];
	tree.clear();
	adjlist.clear();
	openlist.clear();
	closedlist.clear();
}

void generate_random_point(double _x, double _z, mt19937& gen, double& x, double& y, double& z)
{
	//以终点为原点建立球坐标系
	normal_distribution<> r_dist(0, std_dev);
	double r = abs(r_dist(gen));    //正态分布得到极径
	uniform_real_distribution<> phi_dist(-PI / 2, PI / 2);
	double phi = phi_dist(gen);    //均匀分布得到纬度
	uniform_real_distribution<> theta_dist(0, PI);
	double theta = theta_dist(gen);    //均匀分布得到经度（仅取y>0半球）

	//球坐标转笛卡尔坐标
	x = _x + r * cos(phi) * cos(theta);
	y = r * cos(phi) * sin(theta);
	z = r * sin(phi);
}

//检查随机点是否在障碍物中
bool check_pos(double x, double y, double z)
{
	if (y <= 0 || z <= 0) return false;
	else return true;
}

multimap<double, Node*>::iterator in_openlist(Node* ptr)
{
	multimap<double, Node*>::iterator it = openlist.find(heuristic_optimize ? ptr->f : ptr->g);
	while (it != openlist.end() && it->first == (heuristic_optimize ? ptr->f : ptr->g))
	{
		if (it->second == ptr)
			return it;
		it++;
	}
	return openlist.end();
}

Node* route_search()
{
	Node* ptr = tree[0];
	openlist.insert(make_pair(heuristic_optimize ? ptr->f : ptr->g, ptr));
	while(!openlist.empty())
	{
		ptr = openlist.begin()->second;
		openlist.erase(openlist.begin());
		closedlist.insert(ptr);
		if (ptr->x == x_e2 && ptr->y == 0 && ptr->z == z_e2)
			return ptr;
		auto it1 = adjlist.find(ptr);
		while(it1 != adjlist.end() && it1->first == ptr)
		{
			Node* next = it1->second;
			if (closedlist.find(next) == closedlist.end())
			{
				auto it2 = in_openlist(next);
				if (it2 == openlist.end())
				{
					next->g = ptr->g + dist3(ptr->x, ptr->y, ptr->z, next->x, next->y, next->z);
					if (heuristic_optimize)
						next->f = next->g + RRT_W * dist3(next->x, next->y, next->z, x_e2, 0, z_e2);
					next->parent = ptr;
					openlist.insert(make_pair(heuristic_optimize ? next->f : next->g, next));
				}
				else
				{
					if (ptr->g + dist3(ptr->x, ptr->y, ptr->z, next->x, next->y, next->z) < next->g)
					{
					    openlist.erase(it2);
						next->g = ptr->g + dist3(ptr->x, ptr->y, ptr->z, next->x, next->y, next->z);
						if (heuristic_optimize)
							next->f = next->g + RRT_W * dist3(next->x, next->y, next->z, x_e2, 0, z_e2);
						next->parent = ptr;
						openlist.insert(make_pair(heuristic_optimize ? next->f : next->g, next));
					}
				}
			}
			it1++;
		}
	}
	return NULL;
}

RRT_Output RRT_search()
{
	auto start = chrono::high_resolution_clock::now();

	std_dev = std_ratio * dist2(x_e1, z_e1, x_e2, z_e2);
	rrt_n = ((int)(dist2(x_e1, z_e1, x_e2, z_e2) / rrt_step_ratio)) * ITER_RATIO;
	step = rrt_step_ratio * dist2(x_e1, z_e1, x_e2, z_e2);
	neighbor_range = neighbor_range_ratio * step;

	rrt_base_path.reserve(rrt_n);
	rrt_joint_path.reserve(rrt_n);
	rrt_end_path.reserve(rrt_n);

	Node* p = new Node({ x_e1, 0, z_e1, relaxation_optimize ? 0 : MAX_DBL, heuristic_optimize ? dist2(x_e1, z_e1, x_e2, z_e2) : MAX_DBL, NULL });
	tree.push_back(p);
	random_device rd;
	mt19937 gen(rd());
	double cnt = goal_bias;
	int terminal_index = -1;
	
	double x_dir, y_dir, z_dir;
	for (int i = 0; i < rrt_n; i++)
	{
		if (goal_bias_optimize && cnt >= 1)
		{
			//goal_bias的频率下以终点为搜索树生长方向
			x_dir = x_e2;
			y_dir = 0;
			z_dir = z_e2;
			cnt += goal_bias - 1;
		}
		else
		{
			generate_random_point(x_e2, z_e2, gen, x_dir, y_dir, z_dir);
			while (!check_pos(x_dir, y_dir, z_dir)) generate_random_point(x_e2, z_e2, gen, x_dir, y_dir, z_dir);
			cnt += goal_bias;
		}
		if(!relaxation_optimize)
		{
			Node* ptr = tree[0];
			double min = dist3(ptr->x, ptr->y, ptr->z, x_dir, y_dir, z_dir);
			for (int i = 0; i < tree.size(); i++)
			{
				if (dist3(tree[i] -> x, tree[i] -> y, tree[i] -> z, x_dir, y_dir, z_dir) < min)
				{
					ptr = tree[i];
					min = dist3(ptr->x, ptr->y, ptr->z, x_dir, y_dir, z_dir);
				}
			}
			if (min > step)
			{
				x_dir = ptr->x + (x_dir - ptr->x) * step / min;
				y_dir = ptr->y + (y_dir - ptr->y) * step / min;
				z_dir = ptr->z + (z_dir - ptr->z) * step / min;
				if (!check_pos(x_dir, y_dir, z_dir))
				{
					cnt -= goal_bias;
					continue;
				}
			}
			p = new Node({ x_dir, y_dir, z_dir, MAX_DBL, MAX_DBL, ptr });
			tree.push_back(p);
			if (dist3(x_dir, y_dir, z_dir, x_e2, 0, z_e2) < step)
			{
				p = new Node({ x_e2, 0, z_e2, MAX_DBL, MAX_DBL, tree.back() });
				tree.push_back(p);
				break;
			}
		}
		else
		{
			vector<Node*> neighbors;
			Node* ptr = NULL;
			double min = MAX_DBL;
			for (auto it = tree.begin(); it != tree.end(); it++)
			{
				double dist = dist3((*it)->x, (*it)->y, (*it)->z, x_dir, y_dir, z_dir);
				if (dist <= neighbor_range)
					neighbors.push_back(*it);
				if (ptr == NULL)
				{
					ptr = *it;
					min = dist;
				}
				else if (dist < min)
				{
					ptr = *it;
					min = dist;
				}
			}
			if (neighbors.empty())
			{
				x_dir = ptr->x + (x_dir - ptr->x) * step / min;
				y_dir = ptr->y + (y_dir - ptr->y) * step / min;
				z_dir = ptr->z + (z_dir - ptr->z) * step / min;
				if (!check_pos(x_dir, y_dir, z_dir))
				{
					cnt -= goal_bias;
					continue;
				}
				p = new Node({ x_dir, y_dir, z_dir, MAX_DBL, MAX_DBL, NULL });
				tree.push_back(p);
				adjlist.insert(make_pair(ptr, tree.back()));
				adjlist.insert(make_pair(tree.back(), ptr));
			}
			else
			{
				if (x_dir == x_e2 && y_dir == 0 && z_dir == z_e2)
				{
					if (terminal_index == -1)
					{
						terminal_index = (int)tree.size();
						p = new Node({ x_dir, y_dir, z_dir, MAX_DBL, MAX_DBL, NULL });
						tree.push_back(p);
					}
					for (i = 0; i < neighbors.size(); i++)
					{
						adjlist.insert(make_pair(neighbors[i], tree[terminal_index]));
						adjlist.insert(make_pair(tree[terminal_index], neighbors[i]));
					}
				}
				else
				{
					p = new Node({ x_dir, y_dir, z_dir, MAX_DBL, MAX_DBL, NULL });
					tree.push_back(p);
					for (i = 0; i < neighbors.size(); i++)
					{
						adjlist.insert(make_pair(neighbors[i], tree.back()));
						adjlist.insert(make_pair(tree.back(), neighbors[i]));
					}
				}
			}
		}
	}
	vector<Node*> temp;
	temp.reserve(rrt_n);
	Node* ptr;
	if (!relaxation_optimize)
	{
		if (tree.back() -> x != x_e2 || tree.back() -> y != 0 || tree.back() -> z != z_e2)
		{
			auto end = std::chrono::high_resolution_clock::now();
			double t = (chrono::duration_cast<chrono::milliseconds>(end - start).count()) / 1000.0;
			return { false, MAX_DBL, t };
		}
		ptr = tree.back();
	}
	else
	{
		if (terminal_index == -1 || (ptr = route_search()) == NULL)
		{
			auto end = std::chrono::high_resolution_clock::now();
			double t = (chrono::duration_cast<chrono::milliseconds>(end - start).count()) / 1000.0;
			return { false, MAX_DBL, t };
		}
	}

	double l;
	if(!relaxation_optimize)
		l = 0;
	else
		l = ptr->g;
	while (ptr->parent)
	{
		temp.push_back(ptr);
		ptr = ptr->parent;
	}
	temp.push_back(ptr);

	auto end = std::chrono::high_resolution_clock::now();
	double t = (chrono::duration_cast<chrono::milliseconds>(end - start).count()) / 1000.0;

	int sz = (int)temp.size();
	double sum = 0;
	for (int i = sz - 1; i >= 0; i--)
	{
		rrt_end_path.push_back({ temp[i]->x, temp[i]->y, temp[i]->z });
		if (i < sz - 1)
		{
			if(!relaxation_optimize) l += dist3(temp[i]->x, temp[i]->y, temp[i]->z, temp[i + 1]->x, temp[i + 1]->y, temp[i + 1]->z);
			sum += _beam_search(temp[i + 1]->x, temp[i + 1]->y, temp[i + 1]->z, temp[i]->x, temp[i]->y, temp[i]->z).distance_cost;
			for (int j = 0; j < beam_base_path.size() - (i != 0); j++)
			{
				rrt_base_path.push_back(beam_base_path[j]);
				rrt_joint_path.push_back(beam_joint_path[j]);
			}
			reset_beam();
			update_base_position();
		}
	}
	return { true, l, sum, t };
}