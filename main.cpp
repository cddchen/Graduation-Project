#include <bits/stdc++.h>
using namespace std;
const int maxn = 1e3 + 5;
const int update_end_cnt = 1;
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
    friend ostream & operator <<(ostream & os,const point & p){
        return os << "(" << p.x << ", " << p.y << " ," << p.z << ")";
    }
};
void read(point &t) {
    scanf("%d%d%d", &t.x, &t.y, &t.z);
}
typedef pair<int, point> PII;
int n;
//立方体模型
struct node {
    point lb, ru;
} box[maxn];
point st, ed;
//检查点是否出界
bool vis[maxn][maxn][maxn];
bool check_vis(point p) { 
    if (p.x < 0 || p.y < 0 || p.z < 0)
        return true;
    return false;
}
//方向向量
int dir[6][3] = {{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};
//设置距离
int dist[maxn][maxn][maxn];
bool set_dist(point p, point pre) {
    if (dist[p.x][p.y][p.z] > dist[pre.x][pre.y][pre.z] + 1) {
        dist[p.x][p.y][p.z] = dist[pre.x][pre.y][pre.z] + 1;
        return 1;
    }
    return 0;
}
//遍历点序列
point q[maxn];
int front = 0, rear = -1;
int pre[maxn], min_path_length = -1, cnt = 0;
vector<int> ed_idxs;
bool search() {
    cerr << "正在寻找最短路径" << endl;
    memset(dist, 0x3f, sizeof dist);
    dist[st.x][st.y][st.z] = 0;
    vis[st.x][st.y][st.z] = 1;
    q[++rear] = st;
    while (front <= rear) {
        point now = q[front++];
        vis[now.x][now.y][now.z] = 0;
        if (min_path_length != -1 && dist[now.x][now.y][now.z] > min_path_length) continue;
        if (now == ed) {
            cerr << "找到一条最短路径了～" << front - 1 << endl;
            if (min_path_length == -1 || (dist[now.x][now.y][now.z] <= min_path_length)) {
                min_path_length = dist[now.x][now.y][now.z];
                ed_idxs.push_back(front - 1);
            }
            continue;
        }
        
        for (int i = 0; i < 6; ++i) {
            point nx(now.x + dir[i][0], now.y + dir[i][1], now.z + dir[i][2]);
            if (check_vis(nx) || vis[nx.x][nx.y][nx.z]) continue;
            if (!set_dist(nx, now)) continue;
            q[++rear] = nx;
            vis[nx.x][nx.y][nx.z] = 1;
            pre[rear] = front - 1;
        }
    }
    return ed_idxs.size();
}
//路径序列
vector< point > paths;
bool getpath() {
    bool is_find_path = search();
    if (!is_find_path) {
        cerr << "没有找到路径！" << endl;
        return 0;
    }
    cerr << "开始记录路径所经过点" << endl;
    for (auto now : ed_idxs) {
        while (now != 0) {
            paths.push_back(q[now]);
            now = pre[now];
        }
        paths.push_back(q[0]);
        reverse(paths.begin(), paths.end());
        cerr << "路径经过" << paths.size() << "个点。" << endl;
        for (auto i : paths) {
            cerr << i << endl;
        }
    }
    return paths[0] == st && paths.back() == ed;
}
//判断线段与面相交，P+tQ的向量与第i个box相交判断
bool check_segment_cross_obstacle(point P, point Q, int i) {
    int L = box[i].lb.x, R = box[i].ru.x;
    double t1, t2;
    //cerr << "vector " << P << "->" << Q << "insect judge with box" << i << endl;
    if (Q.x == 0) {
        if (!(L <= P.x && P.x <= R))
            return false;
    }
    else {
        t1 = 1.0 * (L - P.x) / Q.x;
        t2 = 1.0 * (R - P.x) / Q.x;
        if (Q.x < 0) swap(t1, t2);
        if (t1 > t2) return false;
    }
    
    L = box[i].lb.y, R = box[i].ru.y;
    if (Q.y == 0) {
        if (!(L<= P.y && P.y <= R))
            return false;
    }
    else {
        t1 = 1.0 * (L - P.y) / Q.y;
        t2 = 1.0 * (R - P.y) / Q.y;
        if (Q.y < 0) swap(t1, t2);
        if (t1 > t2) return false;
    }
    
    L = box[i].lb.z, R = box[i].ru.z;
    if (Q.z == 0) {
        if (!(L <= P.z && P.z <= R))
            return false;
    }
    else {                                   
        t1 = 1.0 * (L - P.z) / Q.z;
        t2 = 1.0 * (R - P.z) / Q.z;
        if (Q.z < 0) swap(t1, t2);
        if (t1 > t2) return false;
    }
    
    return true;
}
int check_segement_cross_all_obstacles(point s, point t) {
    for (int i = 0; i < n; ++i) {
        if (check_segment_cross_obstacle(s, t - s, i))
            return i;
    }
    return -1;
}
//拟合路径 
vector<point> ans;
void calc_path() {
    if (!getpath()) {
        cerr << "未找到路径！" << endl;
        return;
    }
    for (int i = 0; i < paths.size(); ++i) {
        for (int j = 0; j < i; ++j) {
            if (check_segement_cross_all_obstacles(paths[j], paths[i]) == -1) {
                pre[i] = j;
                cerr << "link: " << paths[j] << ' ' << paths[i] << endl;
                break;
            }
        }
    }
    for (int i = paths.size() - 1; i > 0; i = pre[i]) {
        ans.push_back(paths[i]);
    }
    ans.push_back(st);
    cerr << "拟合完毕，路线为：";
    reverse(ans.begin(), ans.end());
    for (auto i : ans) {
        cerr << i << "->";
    }
    cerr << endl;
}
int main()
{
    freopen("in.txt", "r", stdin);
    freopen("out.txt", "w", stdout);
    scanf("%d", &n);
    read(st); read(ed);
    for (int i = 0; i < n; ++i) {
        read(box->lb); read(box->ru);
    }
    //标记不可标记的点
    for (int r = 0; r < n; ++r) {
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
    calc_path();
}