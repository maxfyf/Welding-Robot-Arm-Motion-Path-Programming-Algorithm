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
bool octree_optimize;    //是否八叉树优化
bool relaxation_optimize;    //是否松弛优化
HeuristicFunction heuristic_function;    //启发式函数

vector<Base> rrt_base_path;    //基座路径
vector<Joint> rrt_joint_path;    //关节路径
vector<End> rrt_end_path;    //末端路径

int rrt_n;    //迭代次数
double step;    //搜索树生长步长
double neighbor_range;    //邻域半径

void error();

typedef struct Node
{
	double x;
	double y;
	double z;
	double g;    //代价
	double h;    //启发项
	struct Node* parent;
}Node;    //搜索树上结点
vector<Node*> tree;    //搜索树
bool can_grow(Node* node, double x_dir, double y_dir, double z_dir, double& min)
{
	if (!node) error();
	double dist;
	double x_e, y_e, z_e;
	if ((dist = dist3(node->x, node->y, node->z, x_dir, y_dir, z_dir)) < min)
	{
		if (dist > step)
		{
			x_e = node->x + (x_dir - node->x) * step / dist;
			y_e = node->y + (y_dir - node->y) * step / dist;
			z_e = node->z + (z_dir - node->z) * step / dist;
		}
		else
		{
			x_e = x_dir;
			y_e = y_dir;
			z_e = z_dir;
		}
		if (hit(node->x, node->y, node->z, x_e, y_e, z_e)) return false;
		min = dist;
		return true;
	}
	return false;
}

typedef struct Octnode
{
	double x, y, z;
	double xw;    //法向宽度
	double yw;    //高度
	double zw;    //方向宽度
	struct Octnode* child[8];
	struct Octnode* parent;
	int num;
	Node* nodes[MAX_POINTS_PER_OCTNODE];
}Octnode;    //八叉树上节点
class Octree
{
private:
	Octnode* root;    //八叉树根
	double x_c, y_c, z_c;	//八叉树中心坐标
	double direction;    //方向宽度
	double normal;    //法向宽度
	double height;    //高度

	void transformation(double x0, double y0, double z0, double& x, double& y, double& z)    //从原始坐标系变换到八叉树空间坐标系
	{
		double vec_x = x0 - x_c;
		double vec_y = y0 - y_c;
		double vec_z = z0 - z_c;
		double theta = ((z_e1 == z_e2) ? (x_e1 > x_e2 ? PI / 2 : -PI / 2) : atan((x_e1 - x_e2) / (z_e2 - z_e1)));
		x = vec_x * cos(theta) + vec_z * sin(theta);
		y = vec_y;
		z = vec_z * cos(theta) - vec_x * sin(theta);
	}

	void init_octnode(Octnode* node)
	{
		for (int i = 0; i < 8; i++)
			node->child[i] = NULL;
		node->parent = NULL;
		node->num = 0;
		for (int i = 0; i < MAX_POINTS_PER_OCTNODE; i++)
			node->nodes[i] = NULL;
	}

	Octnode* find_leaf(double x, double y, double z)
	{
		Octnode* node = root;
		if (!root) error();
		while (node->num > MAX_POINTS_PER_OCTNODE)
		{
			int index = 0;
			if (x > node->x) index += 4;
			if (y > node->y) index += 2;
			if (z > node->z) index += 1;
			node = node->child[index];
			if (!node) error();
		}
		return node;
	}

	void split(Octnode* node)
	{
		if (node->num != MAX_POINTS_PER_OCTNODE) error();
		for (int i = 0; i < 8; i++)
		{
			Octnode* p = new Octnode();
			init_octnode(p);
			p->x = (i & 4) ? (node->x + node->xw / 4) : (node->x - node->xw / 4);
			p->y = (i & 2) ? (node->y + node->yw / 4) : (node->y - node->yw / 4);
			p->z = (i & 1) ? (node->z + node->zw / 4) : (node->z - node->zw / 4);
			p->xw = node->xw / 2;
			p->yw = node->yw / 2;
			p->zw = node->zw / 2;
			node->child[i] = p;
			p->parent = node;
		}

		for (int i = 0; i < MAX_POINTS_PER_OCTNODE; i++)
		{
			Node* p = node->nodes[i];
			int index = 0;
			double x, y, z;
			transformation(p->x, p->y, p->z, x, y, z);
			if (x > node->x) index += 4;
			if (y > node->y) index += 2;
			if (z > node->z) index += 1;
			Octnode* child = node->child[index];
			if (!child) error();
			child->nodes[child->num] = p;
			child->num++;
		}
	}

	Node* visit_leaf_nodes_to_update(Octnode* node, double x, double y, double z, double& min, Node* ptr)
	{
		if (node->num > MAX_POINTS_PER_OCTNODE) error();
		for (int i = 0; i < node->num; i++)
		{
			Node* p = node->nodes[i];
			ptr = can_grow(p, x, y, z, min) ? p : ptr;
		}
		return ptr;
	}

	void visit_leaf_nodes_to_connect(Octnode* node, Node* ptr, double nr, vector<Node*>& neighbors)
	{
		if (node->num > MAX_POINTS_PER_OCTNODE) error();
		for (int i = 0; i < node->num; i++)
		{
			Node* p = node->nodes[i];
			if (ptr != p && dist3(ptr->x, ptr->y, ptr->z, p->x, p->y, p->z) <= nr && !hit(ptr->x, ptr->y, ptr->z, p->x, p->y, p->z))
				neighbors.push_back(p);
		}
	}

	Node* traverse_octree_for_node(Octnode* node, double x0, double y0, double z0, double& min, Node* ptr)
	{
		double x, y, z;
		transformation(x0, y0, z0, x, y, z);
		if (dist_point_cuboid(x, y, z, node->x, node->y, node->z, node->xw, node->yw, node->zw) >= min) return ptr;    //剪枝
		if (node->num > MAX_POINTS_PER_OCTNODE)
		{
			for (int i = 0; i < 8; i++)
				ptr = traverse_octree_for_node(node->child[i], x0, y0, z0, min, ptr);
		}
		else
			ptr = visit_leaf_nodes_to_update(node, x0, y0, z0, min, ptr);
		return ptr;
	}

	void traverse_octree_for_neighbors(Octnode* node, Node* ptr, double nr, vector<Node*>& neighbors)
	{
		double x, y, z;
		transformation(ptr->x, ptr->y, ptr->z, x, y, z);
		if (dist_point_cuboid(x, y, z, node->x, node->y, node->z, node->xw, node->yw, node->zw) > nr) return;    //剪枝
		if (node->num > MAX_POINTS_PER_OCTNODE)
		{
			for (int i = 0; i < 8; i++)
				traverse_octree_for_neighbors(node->child[i], ptr, nr, neighbors);
		}
		else
			visit_leaf_nodes_to_connect(node, ptr, nr, neighbors);
	}

	void free_octree(Octnode* node)
	{
		if (!node) return;
		if (node->num > MAX_POINTS_PER_OCTNODE)
		{
			for (int i = 0; i < 8; i++)
			{
				free_octree(node->child[i]);
				node->child[i] = NULL;
			}
		}
		delete node;
	}

public:
	Octree()
	{
		x_c = (x_e1 + x_e2) / 2;
		y_c = h0 / lr / 2;
		z_c = (z_e1 + z_e2) / 2;
		direction = dist2(x_e1, z_e1, x_e2, z_e2);
		normal = direction;
		height = h0 / lr;
		root = new Octnode();
		init_octnode(root);
		root->x = 0;
		root->y = 0;
		root->z = 0;
		root->xw = normal;
		root->yw = height;
		root->zw = direction;
		insert(tree[0]);
	}

	void insert(Node* node)
	{
		double x, y, z;
		transformation(node->x, node->y, node->z, x, y, z);
		Octnode* p = find_leaf(x, y, z);
		while (p->num == MAX_POINTS_PER_OCTNODE)
		{
			split(p);
			p->num++;
			int index = 0;
			if (x > p->x) index += 4;
			if (y > p->y) index += 2;
			if (z > p->z) index += 1;
			p = p->child[index];
			if (!p) error();
		}
		if (p->num >= MAX_POINTS_PER_OCTNODE) error();
		p->nodes[p->num] = node;
		p->num++;
	}

	Node* find_closest(double x0, double y0, double z0, double& min)
	{
		Node* ptr = NULL;
		min = MAX_DBL;

		double x, y, z;
		transformation(x0, y0, z0, x, y, z);
		Octnode* present = find_leaf(x, y, z);
		ptr = visit_leaf_nodes_to_update(present, x0, y0, z0, min, ptr);
		Octnode* prev = present;
		present = present->parent;
		while (present)
		{
			for (int i = 0; i < 8; i++)
			{
				if (present->child[i] != prev)
					ptr = traverse_octree_for_node(present->child[i], x0, y0, z0, min, ptr);
			}
			prev = present;
			present = present->parent;
		}
		return ptr;
	}

	void find_neighbors(Node* node, double nr, vector<Node*>& neighbors)
	{
		traverse_octree_for_neighbors(root, node, nr, neighbors);
	}

	~Octree()
	{
		free_octree(root);
		root = NULL;
	}
};
static Octree *octree;    //八叉树

static multimap<Node*, Node*> adjlist;    //邻接表
static multimap<double, Node*> openlist;    //开放列表
static unordered_set<Node*> closedlist;    //封闭列表

void error()
{
	if (octree)
	{
		delete octree;
		octree = NULL;
	}
	for (int i = 0; i < tree.size(); i++)
		delete tree[i];
	tree.clear();

	adjlist.clear();
	openlist.clear();
	closedlist.clear();

	throw std::runtime_error("RRT Error");
}

void rrt_config(int it, double sr, double gb, double nrr, double w, bool gb_opt, bool o_opt, bool r_opt, HeuristicFunction hf)
{
	rrt_n = it;
	rrt_step_ratio = sr;
	goal_bias = gb;
	neighbor_range_ratio = nrr;
	RRT_W = w;
	goal_bias_optimize = gb_opt;
	octree_optimize = o_opt;
	relaxation_optimize = r_opt;
	if(r_opt) heuristic_function = hf;
	else heuristic_function = HeuristicFunction::NONE;

	octree = NULL;
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

double heuristic(Node* node)
{
	double xh, zh, r;
	switch (heuristic_function)
	{
	case HeuristicFunction::NONE:
		return 0;
	case HeuristicFunction::END:
		return dist3(node->x, node->y, node->z, x_e2, 0, z_e2);
	case HeuristicFunction::BOTTLENECKPOINT:
		projection(node->x, node->z, xh, zh, r);
		if (r < 0.5) return dist3(node->x, node->y, node->z, (x_e1 + x_e2) / 2, (h0 + h0 / lr) / 2, (z_e1 + z_e2) / 2);
		else return -dist3(node->x, node->y, node->z, (x_e1 + x_e2) / 2, (h0 + h0 / lr) / 2, (z_e1 + z_e2) / 2);
	case HeuristicFunction::HYBRID:
		projection(node->x, node->z, xh, zh, r);
		if (r < 0.5) return dist3(node->x, node->y, node->z, (x_e1 + x_e2) / 2, (h0 + h0 / lr) / 2, (z_e1 + z_e2) / 2) + dist3((x_e1 + x_e2) / 2, (h0 + h0 / lr) / 2, (z_e1 + z_e2) / 2, x_e2, 0, z_e2);
		else return dist3(node->x, node->y, node->z, x_e2, 0, z_e2);
	}
}

multimap<double, Node*>::iterator in_openlist(Node* ptr)
{
	multimap<double, Node*>::iterator it = openlist.find(heuristic_function != HeuristicFunction::NONE ? (ptr->g + RRT_W * ptr->h) : ptr->g);
	while (it != openlist.end() && it->first == (heuristic_function != HeuristicFunction::NONE ? (ptr->g + RRT_W * ptr->h) : ptr->g))
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
	openlist.insert(make_pair(heuristic_function != HeuristicFunction::NONE ? (ptr->g + RRT_W * ptr->h) : ptr->g, ptr));
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
					next->parent = ptr;
					openlist.insert(make_pair(heuristic_function != HeuristicFunction::NONE ? (next->g + RRT_W * next->h) : next->g, next));
				}
				else
				{
					if (ptr->g + dist3(ptr->x, ptr->y, ptr->z, next->x, next->y, next->z) < next->g)
					{
					    openlist.erase(it2);
						next->g = ptr->g + dist3(ptr->x, ptr->y, ptr->z, next->x, next->y, next->z);
						next->parent = ptr;
						openlist.insert(make_pair(heuristic_function != HeuristicFunction::NONE ? (next->g + RRT_W * next->h) : next->g, next));
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
	Node* prev_biased_ptr = NULL;
	Node* p = new Node({ x_e1, 0, z_e1, relaxation_optimize ? 0 : MAX_DBL, MAX_DBL, NULL });
	if(heuristic_function != HeuristicFunction::NONE)
		p->h = heuristic(p);
	tree.push_back(p);
	if (octree_optimize) octree = new Octree();
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
		if (!octree_optimize)
		{
			for (int j = 0; j < tree.size(); j++)
				ptr = can_grow(tree[j], x_dir, y_dir, z_dir, min) ? tree[j] : ptr;
		}
		else
			ptr = octree->find_closest(x_dir, y_dir, z_dir, min);
		if (!ptr)
		{
			if (!is_goal_bias)
			{
				cnt -= goal_bias;
				i--;
			}
			continue;
		}
		else if (is_goal_bias)
		{
			if (ptr == prev_biased_ptr) continue;
			prev_biased_ptr = ptr;
		}
		if (min > step)
		{
			x_dir = ptr->x + (x_dir - ptr->x) * step / min;
			y_dir = ptr->y + (y_dir - ptr->y) * step / min;
			z_dir = ptr->z + (z_dir - ptr->z) * step / min;
		}
		p = new Node({ x_dir, y_dir, z_dir, MAX_DBL, MAX_DBL, ptr });
		if(heuristic_function != HeuristicFunction::NONE)
			p->h = heuristic(p);
		tree.push_back(p);
		if (octree_optimize) octree->insert(p);

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
			if(!octree_optimize)
			{
				for (int j = 0; j < tree.size() - 1; j++)
					if (tree[j] != p && dist3(tree[j]->x, tree[j]->y, tree[j]->z, p->x, p->y, p->z) <= neighbor_range && !hit(tree[j]->x, tree[j]->y, tree[j]->z, p->x, p->y, p->z))
						neighbors.push_back(tree[j]);
			}
			else
				octree->find_neighbors(p, neighbor_range, neighbors);
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
	if (octree_optimize) delete octree;

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