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

double rrt_step_ratio;    //搜索步长与起点和终点间距的比值
double goal_bias;    //目标偏置频率
double neighbor_range_ratio;    //邻域半径与搜索步长的比值
double RRT_W;    //启发项权重
bool goal_bias_optimize;	   //是否目标偏置优化
bool relaxation_optimize;    //是否松弛优化
bool heuristic_optimize;    //是否启发式优化

vector<Base> rrt_base_path;    //基座路径
vector<Joint> rrt_joint_path;    //关节路径
vector<End> rrt_end_path;    //末端路径

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

void rrt_config(int it, double sr, double gb, double nrr, double w, bool gb_opt, bool r_opt, bool h_opt)
{
	rrt_n = it;
	rrt_step_ratio = sr;
	goal_bias = gb;
	neighbor_range_ratio = nrr;
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

void generate_random_point(mt19937& gen, double& x, double& y, double& z)
{
	//始末坐标连线周围均匀分布
	uniform_real_distribution<> ratio(0, 1);

	//切向
	double t = ratio(gen);
	double x_t = (x_e2 - x_e1) * t;
	double z_t = (z_e2 - z_e1) * t;

	//法向
	double n = ratio(gen) - 0.5;
	double x_n = (z_e1 - z_e2) * n;
	double z_n = (x_e2 - x_e1) * n;

	x = x_e1 + x_t + x_n;
	z = z_e1 + z_t + z_n;

	//y轴方向
	y = ratio(gen) * h0 / lr;
}

//检查随机点是否在障碍物中
bool check_pos(double x, double y, double z)
{
	if (y < 0 || z < 0) return false;
	if (x == x_e2 && y == 0 && z == z_e2) return false;
	double xh, zh, r;
	projection(x, z, xh, zh, r);
	if (r < (1 - lr) / 2 || r > (1 + lr) / 2) return true;
	else return (y > h0);
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

	step = rrt_step_ratio * dist2(x_e1, z_e1, x_e2, z_e2);
	neighbor_range = neighbor_range_ratio * step;

	rrt_base_path.reserve(rrt_n);
	rrt_joint_path.reserve(rrt_n);
	rrt_end_path.reserve(rrt_n);

	Node* ptr;
	Node* p = new Node({ x_e1, 0, z_e1, relaxation_optimize ? 0 : MAX_DBL, heuristic_optimize ? dist2(x_e1, z_e1, x_e2, z_e2) : MAX_DBL, NULL });
	tree.push_back(p);
	random_device rd;
	mt19937 gen(rd());
	double cnt = goal_bias;
	bool is_goal_bias;
	int terminal_index = -1;
	
	double x_dir, y_dir, z_dir;
	for (int i = 0; i < rrt_n; i++)
	{
		if (goal_bias_optimize && cnt >= 1 && (!relaxation_optimize || terminal_index == -1))
		{
			//goal_bias的频率下以终点为搜索树生长方向
			is_goal_bias = true;
			x_dir = x_e2;
			y_dir = 0;
			z_dir = z_e2;
			cnt += goal_bias - 1;
		}
		else
		{
			is_goal_bias = false;
			do generate_random_point(gen, x_dir, y_dir, z_dir);
			while (!check_pos(x_dir, y_dir, z_dir));
			cnt += goal_bias;
		}

		double min = MAX_DBL;
		ptr = NULL;
		double dist;
		double x_e, y_e, z_e;
		for (int j = 0; j < tree.size(); j++)
		{
			if ((dist = dist3(tree[j]->x, tree[j]->y, tree[j]->z, x_dir, y_dir, z_dir)) < min)
			{
				if (dist > step)
				{
					x_e = tree[j]->x + (x_dir - tree[j]->x) * step / dist;
					y_e = tree[j]->y + (y_dir - tree[j]->y) * step / dist;
					z_e = tree[j]->z + (z_dir - tree[j]->z) * step / dist;
				}
				else 
				{
					x_e = x_dir;
					y_e = y_dir;
					z_e = z_dir;
				}
				if (hit(tree[j]->x, tree[j]->y, tree[j]->z, x_e, y_e, z_e)) continue;
				ptr = tree[j];
				min = dist;
			}
		}
		if (!ptr)
		{
			if (!is_goal_bias)
			{
				cnt -= goal_bias;
				i--;
			}
			continue;
		}
		if (min > step)
		{
			x_dir = ptr->x + (x_dir - ptr->x) * step / min;
			y_dir = ptr->y + (y_dir - ptr->y) * step / min;
			z_dir = ptr->z + (z_dir - ptr->z) * step / min;
		}
		p = new Node({ x_dir, y_dir, z_dir, MAX_DBL, MAX_DBL, ptr });
		tree.push_back(p);

		if(!relaxation_optimize)
		{
			if (dist3(x_dir, y_dir, z_dir, x_e2, 0, z_e2) <= step)
			{
				ptr = p;
				p = new Node({ x_e2, 0, z_e2, MAX_DBL, MAX_DBL, ptr });
				tree.push_back(p);
				break;
			}
		}
		else
		{
			if (x_dir == x_e2 && y_dir == 0 && z_dir == z_e2)
				terminal_index = (int)(tree.size() - 1);
			vector<Node*> neighbors;
			for(int j = 0; j < tree.size() - 1; j++)
				if (dist3(tree[j]->x, tree[j]->y, tree[j]->z, p->x, p->y, p->z) <= neighbor_range && !hit(tree[j]->x, tree[j]->y, tree[j]->z, p->x, p->y, p->z))
					neighbors.push_back(tree[j]);
			if (neighbors.empty())
			{
				adjlist.insert(make_pair(ptr, p));
				adjlist.insert(make_pair(p, ptr));
			}
			else
				for (int j = 0; j < neighbors.size(); j++)
				{
					adjlist.insert(make_pair(neighbors[j], p));
					adjlist.insert(make_pair(p, neighbors[j]));
				}
		}
	}
	vector<Node*> temp;
	temp.reserve(rrt_n);
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

	int sz = (int)temp.size();
	double sum = 0;
	Beam_Output beam_output;
	for (int i = sz - 1; i >= 0; i--)
	{
		rrt_end_path.push_back({ temp[i]->x, temp[i]->y, temp[i]->z });
		if (i < sz - 1)
		{
			if(!relaxation_optimize) l += dist3(temp[i]->x, temp[i]->y, temp[i]->z, temp[i + 1]->x, temp[i + 1]->y, temp[i + 1]->z);
			beam_output = _beam_search(temp[i + 1]->x, temp[i + 1]->y, temp[i + 1]->z, temp[i]->x, temp[i]->y, temp[i]->z);
			if (!beam_output.success) {
				auto end = std::chrono::high_resolution_clock::now();
				double t = (chrono::duration_cast<chrono::milliseconds>(end - start).count()) / 1000.0;
				return { false, 0, MAX_DBL, t };
			}
			sum += beam_output.distance_cost;
			for (int j = 0; j < beam_base_path.size() - (i != 0); j++)
			{
				rrt_base_path.push_back(beam_base_path[j]);
				rrt_joint_path.push_back(beam_joint_path[j]);
			}
			reset_beam();
			update_base_position();
		}
	}

	auto end = std::chrono::high_resolution_clock::now();
	double t = (chrono::duration_cast<chrono::milliseconds>(end - start).count()) / 1000.0;
	return { true, l, sum, t };
}