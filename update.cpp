#include <iostream>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <cstdlib>
#include <vector>
#include <queue>
//#include "/Applications/Polyspace/R2020a/extern/include/engine.h"
using namespace std;
const int maxn = 1e3 + 5;
const int maxm = 1e4 + 5;
const int update_end_cnt = 1;
const int INF = 0x3f3f3f3f;
const double eps = 1e-5;
//#define cerr cout
//记录空间中的点
struct point2DInt {
    int x, y;
    point2DInt() {}
    point2DInt(int x, int y) : x(x), y(y) {}
    bool operator ==(const point2DInt& rhs) const {
        return x == rhs.x && y == rhs.y;
    }
    point2DInt operator -(const point2DInt& rhs) const {
        return point2DInt(x - rhs.x, y - rhs.y);
    }
    int operator *(const point2DInt& rhs) const {
        return x * rhs.y - rhs.x * y;
    }
    bool operator <(const point2DInt& rhs) const  {
        if (x != rhs.x)
            return x < rhs.x;
        return y < rhs.y;
    }
    friend ostream &operator <<(ostream &out, const point2DInt& rhs) {
        return out << "(" << rhs.x << ", " << rhs.y << ")";
    }
};
struct point2D {
    double x, y;
    point2D() {}
    point2D(double x, double y) : x(x), y(y) {}
    bool operator ==(const point2D& rhs) const {
        return x == rhs.x && y == rhs.y;
    }
    point2DInt operator /(double precision) const {
        return point2DInt(x / precision, y / precision);
    }
    point2D operator -(const point2D& rhs) const {
        return point2D(x - rhs.x, y - rhs.y);
    }
    bool operator <(const point2D& rhs) const  {
        if (x != rhs.x)
            return x < rhs.x;
        return y < rhs.y;
    }
};


struct pointInt {
    int x, y, z;
    pointInt() {}
    pointInt(int x, int y, int z) : x(x), y(y), z(z) {}
    bool operator ==(const pointInt& rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
    pointInt operator -(const pointInt& rhs) const {
        return pointInt(x - rhs.x, y - rhs.y, z - rhs.z);
    }
    bool operator <(const pointInt& rhs) const  {
        if (x != rhs.x)
            return x < rhs.x;
        if (y != rhs.y)
            return y < rhs.y;
        return z < rhs.z;
    }
    friend ostream & operator <<(ostream & os,const pointInt & p){
        return os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
    }
};
struct pointDB {
    double x, y, z;
    pointDB() { }
    pointDB(double _x, double _y, double _z) : x(_x), y(_y), z(_z) { }
    friend istream & operator >>(istream &in, pointDB& p){
        in >> p.x >> p.y >> p.z;
        return in;
    }
    pointInt conveyByPrec(double precision) {
        return pointInt(x / precision, y / precision, z / precision);
    }
};
double get_dist(pointInt a, pointInt b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

struct process_data_node {
    point2DInt vec[4];
    int z0, z1;
    point2DInt& operator[](int i) {
        return vec[i];
    }
    friend ostream &operator <<(ostream &out, const process_data_node& rhs) {
        return out << "[" << rhs.vec[0] << ", " << rhs.vec[1] << ", " << rhs.vec[2] << ", " << rhs.vec[3] << "], (" << rhs.z0 << ", " << rhs.z1 << ")";
    }
};
//原始数据
struct node {
    point2D vec[4];
    double z0, z1;
    /*friend ostream & operator <<(ostream & os,const node & p){
        return os << "[" << p.lb << ", " << p.ru << "]";
    }*/
    process_data_node conveyByPrec(double precision) {
        process_data_node node;
        node.vec[0] = vec[0] / precision;
        node.vec[1] = vec[1] / precision;
        node.vec[2] = vec[2] / precision;
        node.vec[3] = vec[3] / precision;
        node.z0 = z0 / precision;
        node.z1 = z1 / precision;
        return node;
    }
    friend istream & operator >>(istream &in, node& p){
        double x0, y0, x1, y1, d;
        in >> x0 >> y0 >> x1 >> y1 >> d >> p.z0 >> p.z1;
        //特判平行的状况
        if (y0 == y1 || x0 == x1) {
            p.vec[0] = {x0, y0 - d / 2};
            p.vec[1] = {x1, y1 - d / 2};
            p.vec[2] = {x1, y1 + d / 2};
            p.vec[3] = {x0, y0 + d / 2};
        }
        else {
            double k = (x1 - x0) / (y1 - y0);
            double delta = (d / 2) / sqrt(1 + k * k);
            
            p.vec[0] = {x0 - delta, y0 + k * delta};
            p.vec[1] = {x1 - delta, y1 + k * delta};
            p.vec[2] = {x1 + delta, y1 - k * delta};
            p.vec[3] = {x0 + delta, y0 - k * delta};
        }
        return in;
    }
};
vector<node> raw_boxs;
int box_size;
pointDB raw_st, raw_out;
double precision;
void read_data() {
    cin >> raw_st >> raw_out;
    cin >> box_size;
    raw_boxs.resize(box_size);
    for (int i = 0; i < box_size; ++i) {
        cin >> raw_boxs[i];
    }
    cin >> precision;
}
pointInt st, ed;
vector<process_data_node> boxs;
int range_x = 0, range_y = 0, range_z = 0;
void process_data() {
    st = raw_st.conveyByPrec(precision);
    ed = raw_out.conveyByPrec(precision);
    boxs.clear();
    for (auto box : raw_boxs) {
        boxs.push_back(box.conveyByPrec(precision));
        process_data_node& the = boxs.back();
        range_x = max(range_x, max(the[0].x, max(the[1].x, max(the[2].x, the[3].x))));
        range_y = max(range_y, max(the[0].y, max(the[1].y, max(the[2].y, the[3].y))));
        range_z = max(range_z, the.z1);
    }
    range_x = max(range_x, max(st.x, ed.x));
    range_y = max(range_y, max(st.y, ed.y));
    range_z = max(range_z, max(st.z, ed.z));
    cerr << st << "->" << ed << endl;
}
bool check_insect(pointInt P, pointInt Q, process_data_node box) {
    double t1 = 0.0, t2 = 1.0;
    pointInt R = Q - P;
    //check the k of z-axis
    if (R.z == 0) {
        if (!(box.z0 <= P.z && P.z <= box.z1))
            return false;
    }
    else {
        double tmp1 = 1.0 * (box.z0 - P.z) / R.z;
        double tmp2 = 1.0 * (box.z1 - P.z) / R.z;
        if (R.z < 0) swap(tmp1, tmp2);
        t1 = max(t1, tmp1);
        t2 = min(t2, tmp2);
    }
    if (t1 > t2) 
        return false;
    //special judge one point situation
    if (R.x == 0 && R.y == 0) {
        point2DInt E(P.x, P.y);
        int tmp1 = ((box[1] - box[0]) * (E - box[0])) * ((box[3] - box[2]) * (E - box[2]));
        int tmp2 = ((box[0] - box[3]) * (E - box[3])) * ((box[2] - box[1]) * (E - box[1]));
        if (tmp1 >= 0 && tmp2 >= 0)
            return true;
        return false;
    }
    //judge ABxAE*CDxCE>=0
    double a1 = 1.0 * (Q.y - P.y) * (box[1].x - box[0].x) - 1.0 * (Q.x - P.x) * (box[1].y - box[0].y);
    double b1 = 1.0 * (P.y - box[0].y) * (box[1].x - box[0].x) - 1.0 * (P.x - box[0].x) * (box[1].y - box[0].y);
    double a2 = 1.0 * (Q.y - P.y) * (box[3].x - box[2].x) - 1.0 * (Q.x - P.x) * (box[3].y - box[2].y);
    double b2 = 1.0 * (P.y - box[2].y) * (box[3].x - box[2].x) - 1.0 * (P.x - box[2].x) * (box[3].y - box[2].y);
    double delta = 1.0 * (a1 * b2 + a2 * b1) * (a1 * b2 + a2 * b1) - 4.0 * a1 * a2 * b1 * b2;
    if (delta < 0) return false;
    if (a1 == 0 && a2 == 0) {
        if (b1 * b2 < 0)
            return false;
    }
    else if (a1 == 0) {
        double tmp = -1.0 * b2 / a2;
        if (a2 > 0) t1 = max(t1, tmp);
        else t2 = min(t2, tmp);
    }
    else if (a2 == 0) {
        double tmp = -1.0 * b1 / a1;
        if (a1 > 0) t1 = max(t1, tmp);
        else t2 = min(t2, tmp);
    }
    else {
        double x1 = (-1.0 * (a1 * b2 + a2 * b1) - sqrt(delta)) / (2.0 * a1 * a2);
        double x2 = (-1.0 * (a1 * b2 + a2 * b1) + sqrt(delta)) / (2.0 * a1 * a2);
        if (x1 > x2) swap(x1, x2);
        t1 = max(t1, x1);
        t2 = min(t2, x2);
    }
    if (t1 > t2) 
        return false;
    //judge DAxDE*BCxBE>=0
    a1 = 1.0 * (Q.y - P.y) * (box[0].x - box[3].x) - 1.0 * (Q.x - P.x) * (box[0].y - box[3].y);
    b1 = 1.0 * (P.y - box[3].y) * (box[0].x - box[3].x) - 1.0 * (P.x - box[3].x) * (box[0].y - box[3].y);
    a2 = 1.0 * (Q.y - P.y) * (box[2].x - box[1].x) - 1.0 * (Q.x - P.x) * (box[2].y - box[1].y);
    b2 = 1.0 * (P.y - box[1].y) * (box[2].x - box[1].x) - 1.0 * (P.x - box[1].x) * (box[2].y - box[1].y);
    delta = 1.0 * (a1 * b2 + a2 * b1) * (a1 * b2 + a2 * b1) - 4.0 * a1 * a2 * b1 * b2;
    if (delta < 0 || a1 * a2 == 0) return false;
    if (a1 == 0 && a2 == 0) {
        if (b1 * b2 < 0)
            return false;
    }
    else if (a1 == 0) {
        double tmp = -1.0 * b2 / a2;
        if (a2 > 0) t1 = max(t1, tmp);
        else t2 = min(t2, tmp);
    }
    else if (a2 == 0) {
        double tmp = -1.0 * b1 / a1;
        if (a1 > 0) t1 = max(t1, tmp);
        else t2 = min(t2, tmp);
    }
    else {
        double x1 = (-1.0 * (a1 * b2 + a2 * b1) - sqrt(delta)) / (2.0 * a1 * a2);
        double x2 = (-1.0 * (a1 * b2 + a2 * b1) + sqrt(delta)) / (2.0 * a1 * a2);
        if (x1 > x2) swap(x1, x2);
        t1 = max(t1, x1);
        t2 = min(t2, x2);
    }
    if (t1 <= t2) return true;
    return false;
}
int check_all_insect(pointInt P, pointInt Q) {
    for (int i = 0; i < boxs.size(); ++i) {
        if (check_insect(P, Q, boxs[i]))
            return i;
    }
    return -1;
}
int dir[6][3] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
namespace Astar {
    vector<pointInt> arr;
    vector<double> dist;
    vector<int> pre;
    vector<pointInt> paths;
    double min_dist;
    struct heap_node {
        double h, g;
        int id;
        heap_node() {}
        heap_node(double _h, double _g, int _id) : h(_h), g(_g), id(_id) {}
        bool operator <(const heap_node& rhs) const {
            return h + g < rhs.h + rhs.g;
        }
        bool operator >(const heap_node& rhs) const {
            return h + g > rhs.h + rhs.g;
        }
    };
    map<pair<int, pair<int, int> >, int> record;
    pair<int, pair<int, int> > point2pair(pointInt p) {
        return make_pair(p.x, make_pair(p.y, p.z));
    }
    priority_queue<heap_node, vector<heap_node>, greater<heap_node> > Q;
    void __init__() {
        arr.clear();
        dist.clear();
        pre.clear();
        record.clear();
        paths.clear();
        while (Q.size()) Q.pop();
    }
    bool __build__() {
        cerr << "A*算法搜索可达路径中。。。" << endl;
        arr.push_back(st); Q.push({0, get_dist(st, ed), 0});
        pre.push_back(-1); dist.push_back(0);
        record[point2pair(st)] = 0;
        while (!Q.empty()) {
            heap_node top = Q.top(); Q.pop();
            pointInt now = arr[top.id];
            //cerr << top.h << " " << top.g << ": " << now << endl;
            if (now == ed) {
                min_dist = top.h;
                return true;
            }
            for (int i = 0; i < 6; ++i) { 
                pointInt nx(now.x + dir[i][0], now.y + dir[i][1], now.z + dir[i][2]);
                if (nx.x < 0 || nx.y < 0 || nx.z < 0) continue;
                if (nx.x > range_x + 1 || nx.y > range_y + 1 || nx.z > range_z + 1) continue;
                if (check_all_insect(now, nx) != -1) continue;
                if (record.count(point2pair(nx)) == 0) {
                    arr.push_back(nx);
                    int id = arr.size() - 1;
                    pre.push_back(top.id);
                    dist.push_back(dist[top.id] + get_dist(now, nx));
                    record[point2pair(nx)] = id;
                    for (int pre_id = 0; pre_id < id; ++pre_id) {
                        if (check_all_insect(arr[pre_id], nx) == -1) if (dist[pre_id] + get_dist(arr[pre_id], nx) < dist[id]) {
                            dist[id] = dist[pre_id] + get_dist(arr[pre_id], nx);
                            pre[id] = pre_id;
                        }
                    }
                    Q.push({dist[id], get_dist(nx, ed), id});
                }
                else {
                    int id = record[point2pair(nx)];
                    bool isup = false;
                    for (int preid = 0; preid < arr.size(); ++preid) if (preid != id && check_all_insect(arr[preid], arr[id]) == -1) {
                        if (dist[preid] + get_dist(arr[preid], arr[id]) < dist[id]) {
                            dist[id] = dist[preid] + get_dist(arr[preid], arr[id]);
                            pre[id] = preid;
                            isup = true;
                        }
                    }
                    /*
                    if (isup)
                        Q.push({dist[id], get_dist(nx, ed), id});
                    */
                }
            }
        }
        return false;
    }
    bool __process__() {
        int ed_idx = record[point2pair(ed)];
        for (int i = ed_idx; i != -1; i = pre[i]) {
            paths.push_back(arr[i]);
        }
        reverse(paths.begin(), paths.end());
        cerr << "找到经过" << paths.size() << "个点的路径";
        for (int i = 0; i < paths.size(); ++i) {
            cout << paths[i] << endl;
            if (!i) cerr << paths[i];
            else cerr << "->" << paths[i];
        }
        cerr << endl;
        cerr << "该条路径的长度为：" << min_dist << endl;
        cerr << "起点到终点的欧式距离为：" << get_dist(st, ed) << endl;
        return true;
    }
    int __main_() {
        __init__();
        bool build_flg = __build__();
        if (!build_flg) {
            cerr << "未找到从" << st << "到" << ed << "的有效路径！" << endl;
            return 0;
        }
        __process__();
        return 1;
    }
}
int main()
{
    freopen("in1.txt", "r", stdin);
    freopen("out1.txt", "w", stdout);
    read_data();
    process_data();
    Astar::__main_();
    //cerr << check_all_insect(st, ed);
}