#include <bits/stdc++.h>
//#include "/Applications/Polyspace/R2020a/extern/include/engine.h"
using namespace std;
const int maxn = 1e3 + 5;
const int update_end_cnt = 1;
const int INF = 0x3f3f3f3f;
//#define cerr cout
//记录空间中的点

struct point {
    int x, y, z;
    point() {}
    point(int x, int y, int z) : x(x), y(y), z(z) {}
    bool operator ==(const point& rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
    point operator -(const point& rhs) const {
        return point(x - rhs.x, y - rhs.y, z - rhs.z);
    }
    bool operator <(const point& rhs) const  {
        if (x != rhs.x)
            return x < rhs.x;
        if (y != rhs.y)
            return y < rhs.y;
        return z < rhs.z;
    }
    friend ostream & operator <<(ostream & os,const point & p){
        return os << "(" << p.x << ", " << p.y << " ," << p.z << ")";
    }
};
double get_dist(point a, point b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}
int get_sqr_dist(point a, point b) {
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
}
void read(point &t) {
    scanf("%d%d%d", &t.x, &t.y, &t.z);
}
int n;
int max_x, max_y, max_z;
//立方体模型
struct node {
    point lb, ru;
} box[maxn];
point st, ed;
//判断线段与面相交，P+tQ的向量与第i个box相交判断
bool check_segment_cross_obstacle(point P, point Q, int i) {
    int L = box[i].lb.x, R = box[i].ru.x;
    double t1 = 0.0, t2 = 1.0;
    //cerr << "vector " << P << "->" << Q << "insect judge with box" << i << endl;
    if (Q.x == 0) {
        if (!(L <= P.x && P.x <= R))
            return false;
    }
    else {
        double tmp1 = 1.0 * (L - P.x) / Q.x;
        double tmp2 = 1.0 * (R - P.x) / Q.x;
        if (Q.x < 0) swap(tmp1, tmp2);
        if (tmp1 > tmp2) return false;
        t1 = max(t1, tmp1);
        t2 = min(t2, tmp2);
    }
    
    L = box[i].lb.y, R = box[i].ru.y;
    if (Q.y == 0) {
        if (!(L<= P.y && P.y <= R))
            return false;
    }
    else {
        double tmp1 = 1.0 * (L - P.y) / Q.y;
        double tmp2 = 1.0 * (R - P.y) / Q.y;
        if (Q.y < 0) swap(tmp1, tmp2);
        if (tmp1 > tmp2) return false;
        t1 = max(t1, tmp1);
        t2 = min(t2, tmp2);
    }
    
    L = box[i].lb.z, R = box[i].ru.z;
    if (Q.z == 0) {
        if (!(L <= P.z && P.z <= R))
            return false;
    }
    else {                                   
        double tmp1 = 1.0 * (L - P.z) / Q.z;
        double tmp2 = 1.0 * (R - P.z) / Q.z;
        if (Q.z < 0) swap(tmp1, tmp2);
        if (tmp1 > tmp2) return false;
        t1 = max(t1, tmp1);
        t2 = min(t2, tmp2);
    }
    if (t1 <= t2)
        return true;
    return false;
}
int check_segement_cross_all_obstacles(point s, point t) {
    for (int i = 0; i < n; ++i) {
        if (check_segment_cross_obstacle(s, t - s, i))
            return i;
    }
    return -1;
}

//检查点是否出界
bool vis[maxn][maxn][maxn];
bool check_vis(point p) { 
    if (p.x < 0 || p.y < 0 || p.z < 0)
        return true;
    if (p.x > max_x + 1 || p.y > max_y + 1 || p.z > max_z + 1)
        return true;
    //if (vis[p.x][p.y][p.z])
        //return true;
    return false;
}
//方向向量      
int dir[6][3] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
point arr[maxn];
int front, rear, before_p[maxn];
double dist[maxn], min_dist;
bool min_dist_flg = false;
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
//记录该点对应数组的位置
map<pair<int, pair<int, int>>, int> record;
pair<int, pair<int, int>> point2pair(point p) {
    return make_pair(p.x, make_pair(p.y, p.z));
}
priority_queue<heap_node, vector<heap_node>, greater<heap_node>> Q;
bool Astar() {
    cerr << "A*算法搜索可达路径中。。。" << endl;
    front = 0, rear = -1;
    arr[++rear] = st; Q.push({0, get_dist(st, ed), 0}); before_p[0] = -1; dist[0] = 0;
    record.clear(); record[point2pair(st)] = 0; vis[st.x][st.y][st.z] = 1;
    while (!Q.empty()) {
        heap_node top = Q.top(); Q.pop();
        point now = arr[top.id];
        //cerr << now << endl;
        if (now == ed) {
            min_dist = top.h;
            return true;
        }
        for (int i = 0; i < 6; ++i) {
            point nx(now.x + dir[i][0], now.y + dir[i][1], now.z + dir[i][2]);
            if (check_vis(nx)) continue;
            if (!vis[nx.x][nx.y][nx.z]) {
                arr[++rear] = nx;
                dist[rear] = 1.0 * INF;
                record[point2pair(nx)] = rear;
                vis[nx.x][nx.y][nx.z] = 1;
                for (int pre = 0; pre < rear; ++pre) {
                    if (dist[pre] + get_dist(arr[pre], nx) < dist[rear]) if (check_segement_cross_all_obstacles(arr[pre], nx) == -1){
                        dist[rear] = dist[pre] + get_dist(arr[pre], nx);
                        before_p[rear] = pre;
                    }
                    
                }
                Q.push({dist[rear], get_dist(nx, ed), rear});
            }
            else if (record.count(point2pair(nx))) {
                int id = record[point2pair(nx)];
                for (int pre = id + 1; pre <= rear; ++pre) if (check_segement_cross_all_obstacles(arr[pre], nx) == -1) {
                    dist[id] = min(dist[id], dist[pre] + get_dist(arr[pre], nx));
                }
            }
        }
    }
    return false;
}
vector< point > paths;
void getpath() {
    bool is_find_path = Astar();
    if (!is_find_path) {
        cerr << "没有找到路径！" << endl;
        return;
    }
    for (int idx = record[point2pair(ed)]; idx != -1; idx = before_p[idx]) {
        paths.push_back(arr[idx]);
    }
    reverse(paths.begin(), paths.end());
    cerr << "找到经过" << paths.size() << "个点的路径";
    for (int i = 0; i < paths.size(); ++i) {
        if (!i) cerr << paths[i];
        else cerr << "->" << paths[i];
    }
    cerr << endl;
    cerr << "该条路径的长度为：" << min_dist << "，起点到终点的欧式距离为：" << get_dist(st, ed) << endl;
}
int main()
{
    freopen("in.txt", "r", stdin);
    freopen("out.txt", "w", stdout);
    scanf("%d", &n);
    read(st); read(ed);
    max_x = max(st.x, ed.x); max_y = max(st.y, ed.y); max_z = max(st.z, ed.z);
    for (int i = 0; i < n; ++i) {
        read(box->lb); read(box->ru);
    }
    //标记不可标记的点
    for (int r = 0; r < n; ++r) {
        max_x = max(max_x, box[r].ru.x), max_y = max(max_y, box[r].ru.y), max_z = max(max_z, box[r].ru.z);
        for (int i = box[r].lb.x; i <= box[r].ru.x; ++i) {
            for (int j = box[r].lb.y; j <= box[r].ru.y; ++j) {
                for (int k = box[r].lb.z; k <= box[r].ru.z; ++k) {
                    vis[i][j][k] = 1;
                }
            }
        }
    }
    cerr << "出发点：" << st << endl;
    cerr << "终点：" << ed << endl;
    getpath();
}