#include <queue>
#include <cstring>
#include <cmath>
#include <algorithm>
#include "map.h"

int** grid = nullptr;
int** dist = nullptr;
Point** parents = nullptr;
const int32_t dx[] = {1, 0, -1, 0, 1, 1, -1, -1};
const int32_t dy[] = {0, 1, 0, -1, 1, -1, 1, -1};

void Map::initArrays()
{
    grid = static_cast<int**>(ps_malloc(MAP_SQUARE_WIDTH * 4));
    dist = static_cast<int**>(ps_malloc(MAP_SQUARE_WIDTH * 4));
    parents = static_cast<Point**>(ps_malloc(MAP_SQUARE_WIDTH * 6));
    for (int i = 0; i < MAP_SQUARE_WIDTH; i++)
    {
        grid[i] = static_cast<int*>(ps_malloc(MAP_SQUARE_WIDTH * 4));
        dist[i] = static_cast<int*>(ps_malloc(MAP_SQUARE_WIDTH * 4));
        parents[i] = static_cast<Point*>(ps_malloc(MAP_SQUARE_WIDTH * 6));
        memset(grid[i], 0, MAP_SQUARE_WIDTH);
        memset(dist[i], -1, MAP_SQUARE_WIDTH);
        memset(parents[i], -1, MAP_SQUARE_WIDTH);
    }
}

void Map::freeMemory()
{
    for (int i = 0; i < MAP_SQUARE_WIDTH; i++)
    {
        delete[] grid[i];
        delete[] dist[i];
        delete[] parents[i];
    }

    delete[] grid;
    delete[] dist;
    delete[] parents;
}

psvec<Point> Map::a_star(psvec<Point>* obstacles, Point source, Point target)
{
    for (int32_t i = 0; i < obstacles->size(); i++)
        grid[(*obstacles)[i].x][(*obstacles)[i].y] = OBSTACLE;

    dist[source.x][source.y] = 0;
    parents[source.x][source.y] = source;

    std::priority_queue<std::pair<int, Point>, psvec<std::pair<int, Point>>, std::greater<std::pair<int, Point>>> pq;
    pq.push({0, source});

    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();

        if (u == target)
            break;

        for (int i = 0; i < 8; i++)
        {
            int nx = u.x + dx[i];
            int ny = u.y + dy[i];

            if (nx < 0 || ny < 0 || nx >= MAP_SQUARE_WIDTH || ny >= MAP_SQUARE_WIDTH)
                continue;

            if (grid[nx][ny] == 1)
                continue;

            int nd = d + sqrt(pow(nx - u.x, 2) + pow(ny - u.y, 2));

            if (dist[nx][ny] == -1 || nd < dist[nx][ny])
            {
                dist[nx][ny] = nd;
                parents[nx][ny] = u;
                pq.push({nd, {nx, ny}});
            }
        }
    }

    psvec<Point> path;
    Point cur = target;
    while (cur != source)
    {
        if (parents[cur.x][cur.y].x == -1 && parents[cur.x][cur.y].y == -1)
        {
            path.clear();
            return path;
        }
        path.push_back(cur);
        cur = parents[cur.x][cur.y];
    }
    path.push_back(source);
    reverse(path.begin(), path.end());

    return path;
}
