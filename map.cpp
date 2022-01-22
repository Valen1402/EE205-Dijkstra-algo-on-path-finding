#include <vector>
#include <string>
#include <iostream>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include "map.h"

#define MAP_SIZE 10000u

using namespace std;

// Return an equally-distributed random non-negative integer in [0, max_val-1].
static size_t rand_number(size_t max_val)
{
    size_t val = max_val + 1u;
    while (val > max_val) {
        val = rand() / ((RAND_MAX + 1u) / max_val);
    }
    return val;
}

// Return true with probability `prob_true`, otherwise return false.
static bool rand_bool(float prob_true)
{
   return (float)rand() <= prob_true * (float)RAND_MAX;
}

// Recursive DFS to get all reachable crossroads in bitmap.
// seen:   bitmap to check CIDs that has been reached. (input & output)
// cm:     connectivity matrix. (input)
// ncross: # of crossroads. (input)
// trv:    list of CIDs to traverse. (input)
static void dfs(bool *seen, bool *cm, size_t ncross,
                vector<size_t>& trv)
{
    if (trv.empty())
        return;
    vector<size_t> next_trv;
    vector<size_t>::iterator it = trv.begin();
    for (; it < trv.end(); ++it) {
        for (size_t i = 0; i < ncross; ++i) {
            if (i == *it)
                continue;
            if (cm[(*it) * ncross + i] && !seen[i]) {
                seen[i] = true;
                next_trv.push_back(i);
            }
        }
    }
    dfs(seen, cm, ncross, next_trv);
}

namespace map {

// Create a random map with `ncross` crossroads and `nclient` clients.
void create_map(struct Map *map, size_t ncross, size_t nclient)
{
    if (map == 0)
        throw runtime_error("invalid map pointer.");
    if (ncross > MAP_SIZE * MAP_SIZE)
        throw runtime_error("too large value given for `ncross`.");
    srand(time(0));
    // For checking dupication of crossroads.
    bool *bm0 = (bool *)calloc(MAP_SIZE * MAP_SIZE, sizeof(bool));
    if (bm0 == 0)
        throw runtime_error("calloc failed.");
    // Create crossroads at random coordinates avoiding duplication.
    size_t i, j, x, y;
    for (i = 0; i < ncross; ++i) {
        x = rand_number(MAP_SIZE);
        y = rand_number(MAP_SIZE);
        while (bm0[x * MAP_SIZE + y]) {
            x = rand_number(MAP_SIZE);
            y = rand_number(MAP_SIZE);
        }
        bm0[x * MAP_SIZE + y] = true;
        map->crosses.push_back({ .cid = i, .x = (float)x, .y = (float)y });
    }
    free(bm0);
    float ix, dx;
    float iy, dy;
    float sum, rsum;
    // Store connecting probabilities calculated.
    float *probs = (float *)calloc(ncross * ncross, sizeof(float));
    if (probs == 0)
        throw runtime_error("malloc failed.");
    for (i = 0; i < ncross; ++i) {
        // Get the coordinate of `i`.
        ix = map->crosses[i].x;
        iy = map->crosses[i].y;
        for (j = i + 1; j < ncross; ++j) {
            // Calculate probability to connect `i` and `j`.
            // Far apart crossroads have lower probability to be connected.
            dx = map->crosses[j].x - ix;
            dy = map->crosses[j].y - iy;
            float p = 1.0f / (dx * dx + dy * dy);
            p *= p;
            probs[i * ncross + j] = p;
            probs[j * ncross + i] = p;
        }
    }
    // Scale probabilities so that expected # of roads generated
    // for each crossroad in an iteration becomes 1.
    for (i = 0; i < ncross; ++i) {
        sum = 0;
        for (j = 0; j < ncross; ++j) {
            // No self-connection.
            if (i == j)
                continue;
            sum += probs[i * ncross + j];
        }
        // As we set a larger value than 1.0f, it reduces the # of iterations
        // needed until the entire graph becomes connected, but if the value is
        // too large, it may make the graph too dense.
        rsum = 1.0f / sum;
        for (j = 0; j < ncross; ++j) {
            if (i == j)
                continue;
            probs[i * ncross + j] *= rsum;
        }
    }
    // Repeat until the entire graph becomes connected.
    // Connectivity matrix for checking road dupication.
    bool *cm = (bool *)calloc(ncross * ncross, sizeof(bool));
    if (cm == 0)
        throw runtime_error("calloc failed.");
    // Array for graph connectivity check.
    bool *seen = (bool *)calloc(ncross, sizeof(bool));
    if (seen == 0)
        throw runtime_error("calloc failed.");
    bool connected = false;
    vector<size_t> root;
    root.push_back(0);
    while (!connected) {
        // Random road generation.
        for (i = 0; i < ncross; ++i) {
            // Randomly generate roads.
            for (j = 0; j < ncross; ++j) {
                if (i == j)
                    continue;
                if (rand_bool(probs[i * ncross + j])) {
                    // Skip if it already exists.
                    if (!cm[i * ncross + j]) {
                        cm[i * ncross + j] = true;
                        cm[j * ncross + i] = true;
                        map->roads.push_back({ .cids = {i, j} });
                    }
                }
            }
        }
        // Check if the entire graph is connected.
        bool flag = true;
        dfs(seen, cm, ncross, root);
        for (i = 0; i < ncross; ++i) {
            if (!seen[i]) {
                flag = false;
                memset(seen, false, ncross);
                break;
            }
        }
        connected = flag;
    }
    free(cm);
    free(seen);
    free(probs);
    // Create random clients.
    size_t src, dst;
    for (i = 0; i < nclient; ++i) {
        src = rand_number(ncross);
        dst = rand_number(ncross);
        while (src == dst) {
            dst = rand_number(ncross);
        }
        map->clients.push_back({ .src_cid = src, .dst_cid = dst });
    }
}

// Store map as a file.
void store_map(struct Map *map)
{
    if (map == 0)
        throw runtime_error("invalid pointer to map.");
    // Write crosses
    cout << "# " << map->crosses.size() << endl;
    vector<struct Crossroad>::iterator itn = map->crosses.begin();
    for (; itn < map->crosses.end(); ++itn)
        cout << itn->cid << ' ' << itn->x << ' ' << itn->y << endl;
    // Write roads
    cout << "# " << map->roads.size() << endl;
    vector<struct Road>::iterator itr = map->roads.begin();
    for (; itr < map->roads.end(); ++itr)
        cout << itr->cids[0] << ' ' << itr->cids[1] << endl;
    // Write clients
    cout << "# " << map->clients.size() << endl;
    vector<struct Client>::iterator itc = map->clients.begin();
    for (; itc < map->clients.end(); ++itc)
        cout << itc->src_cid << ' ' << itc->dst_cid << endl;
}

// Load map from a file.
void load_map(struct Map *map)
{
    if (map == 0)
        throw runtime_error("invalid pointer to map.");
    map->crosses.clear();
    map->roads.clear();
    map->clients.clear();
    char c;
    size_t len;
    // Read crosses
    cin >> c >> len;
    for (size_t i = 0; i < len; ++i) {
        size_t cid, x, y;
        cin >> cid >> x >> y;
        map->crosses.push_back({ .cid = cid, .x = (float)x, .y = (float)y });
    }
    // Read roads
    cin >> c >> len;
    for (size_t i = 0; i < len; ++i) {
        size_t cid0, cid1;
        cin >> cid0 >> cid1;
        map->roads.push_back({ .cids = {cid0, cid1} });
    }
    // Read clients
    cin >> c >> len;
    for (size_t i = 0; i < len; ++i) {
        size_t src, dst;
        cin >> src >> dst;
        map->clients.push_back({ .src_cid = src, .dst_cid = dst });
    }
}

void store_path(struct Path *path)
{
    if (path == 0)
        throw runtime_error("invalid pointer to path.");
    vector<vector<size_t>>::iterator it = path->paths.begin();
    for (; it < path->paths.end(); ++it) {
        while (it->size() > 1) {
            cout << it->back() << " ";
            it->pop_back();
        }
        cout << it->back() << endl;
    }
}

} // namespace map