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
        return os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
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
    friend ostream & operator <<(ostream & os,const node & p){
        return os << "[" << p.lb << ", " << p.ru << "]";
    }
} box[maxn];
point st, ed;
//判断线段与面相交，P+tQ的向量与第i个box相交判断
bool check_segment_cross_obstacle(point P, point Q, int i) {
    int L = box[i].lb.x, R = box[i].ru.x;
    double t1 = 0.0, t2 = 1.0;
    //cerr << "vector " << P << "->" << Q << "insect judge with box" << box[i] << endl;
    if (Q.x == 0) {
        if (!(L <= P.x && P.x <= R))
            return false;
    }
    else {
        double tmp1 = 1.0 * (L - P.x) / Q.x;
        double tmp2 = 1.0 * (R - P.x) / Q.x;
        if (Q.x < 0) swap(tmp1, tmp2);
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
        t1 = max(t1, tmp1);
        t2 = min(t2, tmp2);
    }
    //cerr << t1 << ' ' << t2 << endl;
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
bool Vis[maxn][maxn][maxn], vis[maxn][maxn][maxn];
bool check_outofbound(point p) { 
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
//A*算法步骤

namespace Astar {
    point arr[maxm];
    int front, rear, before_p[maxm]; //最短路径前溯节点
    double dist[maxm], min_dist;
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
    map<pair<int, pair<int, int> >, int> record;
    pair<int, pair<int, int> > point2pair(point p) {
        return make_pair(p.x, make_pair(p.y, p.z));
    }
    priority_queue<heap_node, vector<heap_node>, greater<heap_node> > Q;
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
                if (check_outofbound(nx)) continue;
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
            cout << paths[i] << endl;
            if (!i) cerr << paths[i];
            else cerr << "->" << paths[i];
        }
        cerr << endl;
        cerr << "该条路径的长度为：" << min_dist << "，起点到终点的欧式距离为：" << get_dist(st, ed) << endl;
    }
}
namespace BFS {
    int dist[maxn][maxn][maxn];
    bool set_dist(point p, point pre) {
        if (dist[p.x][p.y][p.z] > dist[pre.x][pre.y][pre.z] + 1) {
            dist[p.x][p.y][p.z] = dist[pre.x][pre.y][pre.z] + 1;
            return true;
        }
        return false;
    }
    point q[maxm];
    int front, rear;
    int pre[maxm], min_dist;
    bool min_dist_flg;
    vector<int> ed_idxs;
    bool bfs() {
        cerr << "正在搜索最短路径。。。" << endl;
        memset(dist, 0x3f, sizeof(dist));
        dist[st.x][st.y][st.z] = 0;
        vis[st.x][st.y][st.z] = 1;
        front = 0; rear = -1;
        q[++rear] = st; pre[0] = -1;
        min_dist_flg = false;
        ed_idxs.clear();
        while (front <= rear) {
            point now = q[front++];
            cerr << front << ' ' << now << endl;
            vis[now.x][now.y][now.z] = 0;
            if (min_dist_flg && dist[now.x][now.y][now.z] > min_dist) continue;
            if (now == ed) {
                if (!min_dist_flg || dist[now.x][now.y][now.z] <= min_dist) {
                    min_dist_flg = true;
                    min_dist = dist[now.x][now.y][now.z];
                    ed_idxs.push_back(front - 1);
                }
                continue;
            }
            
            for (int i = 0; i < 6; ++i) {
                point nx(now.x + dir[i][0], now.y + dir[i][1], now.z + dir[i][2]);
                if (check_outofbound(nx)) continue;
                if (vis[nx.x][nx.y][nx.z]) continue;
                if (!set_dist(nx, now)) continue;
                q[++rear] = nx;
                vis[nx.x][nx.y][nx.z] = 1;
                pre[rear] = front - 1;
            }
        }
        return ed_idxs.size() != 0;
    }
    
    vector<point> path;
    double optimal_dist; bool optimal_dist_flg;
    
    vector<double> dp_dist;
    vector<int> tmp, tmp_pre;
    void getpath() {
        if (!bfs()) {
            cerr << "未找到路径！" << endl;
            return;
        }
        cerr << "共找到" << ed_idxs.size() << "条路径！开始拟合..." << endl;
        optimal_dist_flg = false;
        for (int k = 0; k < ed_idxs.size(); ++k) {
            int ed_idx = ed_idxs[k];
            cerr << "正在拟合第" << k + 1 << "条路径..." << endl;
            tmp.clear();
            for (int idx = ed_idx; idx != -1; idx = pre[idx]) {
                tmp.push_back(idx);
            }
            reverse(tmp.begin(), tmp.end());
            dp_dist.clear();
            dp_dist.resize(tmp.size());
            tmp_pre.clear();
            tmp_pre.resize(tmp.size());
            for (int i = 0; i < tmp.size(); ++i) {
                if (!i) dp_dist[i] = 0;
                else dp_dist[i] = dp_dist[i - 1] + get_dist(q[tmp[i]], q[tmp[i - 1]]);
                tmp_pre[i] = i - 1;
                
                for (int j = 0; j < i; ++j) {
                    if (check_segement_cross_all_obstacles(q[tmp[i]], q[tmp[j]]) == -1) {
                        if (dp_dist[j] + get_dist(q[tmp[i]], q[tmp[j]]) < dp_dist[i]) {
                            dp_dist[i] = dp_dist[j] + get_dist(q[tmp[i]], q[tmp[j]]);
                            tmp_pre[i] = j;
                        }
                    }
                }
            }
            cerr << "拟合完毕，该条路径长度为：" << dp_dist[tmp.size() - 1] << endl;
            if (!optimal_dist_flg || dp_dist[tmp.size() - 1] < optimal_dist) {
                optimal_dist_flg = true;
                optimal_dist = dp_dist[tmp.size() - 1];
                path.clear();
                for (int i = tmp.size() - 1; i != -1; i = tmp_pre[i]) {
                    path.push_back(q[tmp[i]]);
                }
                reverse(path.begin(), path.end());
            }
        }
        cerr << "找到经过" << path.size() << "个点的路径";
        for (int i = 0; i < path.size(); ++i) {
            if (!i) cerr << path[i];
            else cerr << "->" << path[i];
        }
        cerr << endl;
        cerr << "该条路径的长度为：" << optimal_dist << "，起点到终点的欧式距离为：" << get_dist(st, ed) << endl;
    }
}

int main()
{
    freopen("in.txt", "r", stdin);
    freopen("out.txt", "w", stdout);
    scanf("%d", &n);
    read(st); read(ed);
    max_x = max(st.x, ed.x); max_y = max(st.y, ed.y); max_z = max(st.z, ed.z);
    for (int i = 0; i < n; ++i) {
        read(box[i].lb); read(box[i].ru);
    }
    //标记不可标记的点
    for (int r = 0; r < n; ++r) {
        max_x = max(max_x, box[r].ru.x), max_y = max(max_y, box[r].ru.y), max_z = max(max_z, box[r].ru.z);
        for (int i = box[r].lb.x; i <= box[r].ru.x; ++i) {
            for (int j = box[r].lb.y; j <= box[r].ru.y; ++j) {
                for (int k = box[r].lb.z; k <= box[r].ru.z; ++k) {
                    Vis[i][j][k] = 1;
                }
            }
        }
    }
    cerr << "出发点：" << st << endl;
    cerr << "终点：" << ed << endl;
    //cerr << check_segement_cross_all_obstacles(st, ed) << endl;
    memcpy(vis, Vis, sizeof(Vis));
    Astar::getpath();
    //memcpy(vis, Vis, sizeof(Vis));
    //BFS::getpath();
}