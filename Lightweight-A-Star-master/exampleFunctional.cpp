#include <iostream>
#include <vector>
#include <queue>
#include <cstring>
#include <cmath>
#include <algorithm>

using namespace std;

class Point {
public:
    int x, y;
    Point() {}
    Point(int inx, int iny) { x = inx; y = iny; }

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Point& other) const {
        return x != other.x || y != other.y;
    }

    bool operator<(const Point& other) const {
        if (x == other.x) {
            return y < other.y;
        }
        return x < other.x;
    }
};

const int N = 32;
int n, m;
int grid[N][N];
int dist[N][N];
Point parents[N][N];
const int dx[] = {1, 0, -1, 0, 1, 1, -1, -1};
const int dy[] = {0, 1, 0, -1, 1, -1, 1, -1};

std::vector<Point>* a_star(Point source, Point target) {
    memset(dist, -1, sizeof(dist));
    std::memset(parents, -1, sizeof(parents));
    dist[source.x][source.y] = 0;
    parents[source.x][source.y] = source;


    priority_queue<pair<int, Point>, vector<pair<int, Point>>, greater<pair<int, Point>>> pq;
    pq.push({0, source});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        printf("pq top first is %d, pq size is %d\n", pq.top().first, pq.size());
        pq.pop();

        if (u == target) {
            break;
        }

        for (int i = 0; i < 8; i++) {
            int nx = u.x + dx[i];
            int ny = u.y + dy[i];

            if (nx < 0 || ny < 0 || nx >= n || ny >= m) {
                continue;
            }

            if (grid[nx][ny] == 1) {
                continue;
            }

            int dx = abs(nx - target.x);
            int dy = abs(ny - target.y);
            int nd = d + abs(nx - u.x) + abs(ny - u.y);

            if (dist[nx][ny] == -1 || nd < dist[nx][ny]) {
                dist[nx][ny] = nd;
                parents[nx][ny] = u;
                pq.push({nd, {nx, ny}});
            }
        }
    }

    vector<Point>* path = new vector<Point>();
    Point cur = target;
    while (cur != source) {
        if (parents[cur.x][cur.y].x == -1 && parents[cur.x][cur.y].y == -1) {
            // No valid parent found, path is unreachable
            return nullptr;
        }
        path->push_back(cur);
        cur = parents[cur.x][cur.y];
    }

    path->push_back(source);

    reverse(path->begin(), path->end());

    return path;
}

int main() {
    vector<Point> path;
    n = N, m = N;
    memset(grid, 0, sizeof(grid));
    grid[3][3] = 1;
    printf("amogus\n");
    std::vector<Point>* cock = a_star({31,0}, {0,31});
    printf("sugoma");

    for (int i = 0; i < cock->size(); i++)
    {
        printf("(%d, %d)", (*cock)[i].x, (*cock)[i].y);
    }

    return 0;
}
