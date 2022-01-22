#include <stdio.h>
#include "map.h"

int main(int argc, const char *argv[])
{
    if (argc != 3)
        return 1;
    struct map::Map m;
    map::create_map(&m, atoi(argv[1]), atoi(argv[2]));
    map::store_map(&m);
    return 0;
}