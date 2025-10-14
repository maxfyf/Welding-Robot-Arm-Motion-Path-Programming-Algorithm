#include "global.h"
#include "Beam.h"
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <chrono>
using namespace std;

int K;    //束宽
double W1, W2;    //启发函数中机械臂夹角与基座偏移角的权重 
double lbound, ubound;    //基座在x或y方向位移搜索下限与上限
double step;    //搜索步长
bool optimize;	//是否优化

vector<Base> base_path;    //基座路径
vector<Joint> joint_path;    //关节路径

int n;    //离散化小线段数
int v;    //单次搜索空间大小

typedef struct
{
    double x0, y0;
    int parent_indice;
}Node;    //搜索树上结点

double h(double x0, double y0, double x, double y, double z, double _x, double _z)
{
	double included_angle = calculate_included_angle(x0, y0, x, y, z, _x, _z);
	double bias_angle = calculate_bias_angle(x0, y0, _x);
	return W1 * included_angle + W2 * bias_angle;
}

BeamOutput beam_search(int k, double w1, double w2, double l, double u, double s, bool opt)
{
    auto start = chrono::high_resolution_clock::now();

    K = k;
    W1 = w1;
    W2 = w2;
    lbound = l;
    ubound = u;
    step = s;
    optimize = opt;

    int least_times = (int)(abs(x_e1 - x_e2) / 0.1);
	n = least_times * 10;
	v = (int)((ubound - lbound) / step + 1) * (int)((ubound - lbound) / step + 1);

    int size = 1;    //可能性空间大小
    double _x = x_e1, _z = z_e1;    //焊点实时坐标
	vector<double> x0, y0;    //基座实时坐标
    vector<double> x, y, z;    //关节实时坐标
	vector<double> c;    //c为代价函数
    vector<Node> tree;    //搜索树
    vector<int> tree_index;
    x0.reserve(K);
    y0.reserve(K);
	x.reserve(K);
	y.reserve(K);
	z.reserve(K);
	c.reserve(K);
	tree_index.reserve(K);
    x0.push_back(x_b1);
    y0.push_back(y_b1);
    c.push_back(0);
	Node head = { x0[0], y0[0], -1 };
	tree.push_back(head);
    tree_index.push_back(0);
    x.push_back(0);
    y.push_back(0);
	z.push_back(0);
    calculate_joint_position(x0[0], y0[0], _x, _z, x[0], y[0], z[0]);
    for (int i = 1; i < K; i++)
    {
        x0.push_back(0);
        y0.push_back(0);
		x.push_back(0);
        y.push_back(0);
		z.push_back(0);
		c.push_back(1e9);
		tree_index.push_back(-1);
    }

	double delta_x = (x_e2 - x_e1) / n;    //焊点x方向增量
	double delta_z = (z_e2 - z_e1) / n;    //焊点y方向增量
    double dx;
    double dy;
	vector<double> new_x0, new_y0, new_x, new_y, new_z;
    vector<double> g, f, temp;
    vector<int> parent;
    new_x0.reserve(K * v);
    new_y0.reserve(K * v);
    new_x.reserve(K * v);
    new_y.reserve(K * v);
    new_z.reserve(K * v);
    g.reserve(K * v);
	f.reserve(K * v);
    temp.reserve(K * v);
    parent.reserve(K * v);

    for (int i = 0; i < n; i++)
    {
        _x += delta_x;
        _z += delta_z;
        for (int j = 0; j < size; j++)
        {
            int indice = tree_index[j];
            for (dx = lbound; dx <= ubound; dx += step)
                for (dy = lbound; dy <= ubound; dy += step)
                {
                    new_x0.push_back(x0[j] + dx);
                    new_y0.push_back(y0[j] + dy);
                    new_x.push_back(0);
                    new_y.push_back(0);
                    new_z.push_back(0);
                    if (new_y.back() < 0 || !calculate_joint_position(new_x0.back(), new_y0.back(), _x, _z, new_x.back(), new_y.back(), new_z.back()) || new_y0.back() < 0)    //碰撞检查
                    {
                        new_x0.pop_back();
                        new_y0.pop_back();
                        new_x.pop_back();
                        new_y.pop_back();
                        new_z.pop_back();
                        continue;
                    }
                    g.push_back(c[j] + dist2(new_x0.back(), new_y0.back(), x0[j], y0[j]) + dist3(new_x.back(), new_y.back(), new_z.back(), x[j], y[j], z[j]));
                    if (optimize)
                    {
                        f.push_back(g.back() + h(new_x0.back(), new_y0.back(), new_x.back(), new_y.back(), new_z.back(), _x, _z));
                        temp.push_back(f.back());
                    }
                    else temp.push_back(g.back());
                    parent.push_back(indice);
                }
        }
		size = g.size() > K ? K : (int)g.size();
        if (size == K)
        {
            nth_element(temp.begin(), temp.begin() + size - 1, temp.end());
            double threshold = temp[size - 1];
            int cnt = 0;
            for (int j = 0; j < g.size(); j++)
                if (!optimize && g[j] < threshold || optimize && f[j] < threshold)
                {
                    x0[cnt] = new_x0[j];
                    y0[cnt] = new_y0[j];
                    x[cnt] = new_x[j];
                    y[cnt] = new_y[j];
                    z[cnt] = new_z[j];
                    c[cnt] = g[j];
                    tree_index[cnt] = (int)tree.size();
					tree.push_back({ x0[cnt], y0[cnt], parent[j] });
                    cnt++;
                }
            size = cnt;
        }
        else
        {
            for (int j = 0; j < size; j++)
            {
                x0[j] = new_x0[j];
                y0[j] = new_y0[j];
                x[j] = new_x[j];
                y[j] = new_y[j];
                z[j] = new_z[j];
                c[j] = g[j];
                tree_index[j] = (int)tree.size();
                tree.push_back({ x0[j], y0[j], parent[j] });
            }
        }
		new_x0.clear();
		new_y0.clear();
		new_x.clear();
		new_y.clear();
		new_z.clear();
		g.clear();
        f.clear();
        temp.clear();
        parent.clear();
    }

    //终止
    int indice = 0;
    double min = c[0];
    for (int i = 1; i < size; i++)
        if (c[i] < min)
        {
            indice = i;
            min = c[i];
        }
    vector<int> index_path;
    index_path.push_back(tree_index[indice]);
	while (tree[index_path.back()].parent_indice != -1)
		index_path.push_back(tree[index_path.back()].parent_indice);
    for (int i = (int)index_path.size() - 1; i >= 0; i--)
    {
        base_path.push_back({ tree[index_path[i]].x0, tree[index_path[i]].y0 });
        double x_f, y_f, z_f;
		calculate_joint_position(tree[index_path[i]].x0, tree[index_path[i]].y0, x_e1 + (x_e2 - x_e1) * (index_path.size() - 1 - i) / (index_path.size() - 1), z_e1 + (z_e2 - z_e1) * (index_path.size() - 1 - i) / (index_path.size() - 1), x_f, y_f, z_f);
		joint_path.push_back({ x_f, y_f, z_f });
    }
	x_b2 = base_path.back().x;
	y_b2 = base_path.back().y;

    auto end = std::chrono::high_resolution_clock::now();
    double t = (chrono::duration_cast<chrono::milliseconds>(end - start).count()) / 1000.0;

    return {true, min, t};
}