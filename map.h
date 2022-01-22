#ifndef MAP_H
#define MAP_H

#include <cstdlib>
#include <vector>
#include <string>

namespace map {

struct Crossroad {
    // CID: unique ID of crossroad starting from 0.
    // Initial position of the taxi is CID 0.
    size_t cid;
    // Coordinate (x, y) of crossroad.
    // If `cid` is different, coordinate is also different.
    float x;
    float y;
};

struct Road {
    // Road connects two crossroads with CID `cids[0]` and `cids[1]` each.
    // Road is a bi-directional and unique edge between the crossroads.
    size_t cids[2];
};

struct Client {
    // Client should ride the taxi at `src_cid` crossroad and go to `dst_cid`.
    // `src_cid` and `dst_cid` are always not equal.
    size_t src_cid;
    size_t dst_cid;
};

struct Map {
    // List of all crossroads in the map.
    std::vector<struct Crossroad> crosses;
    // List of all roads in the map.
    std::vector<struct Road> roads;
    // List of all clients in the map.
    std::vector<struct Client> clients;
};

struct Path {
    // List of list of CIDs in a path in reversed order.
    std::vector<std::vector<size_t>> paths;
};

void create_map(struct Map *map, size_t ncross, size_t nclient);
void store_map(struct Map *map);
void load_map(struct Map *map);

void store_path(struct Path *path);

} // namespace map

#endif // MAP_H