#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <iomanip>
#include <fstream>

#include <math.h>

#include "mapping.h"
#include "hash_map.h"
#include "point.h"

class Pathfinding
{
private:

public:
    Pathfinding(/* args */) = default;
    ~Pathfinding() = default;
    void flood_fill(Mapping mapp, Point start, Point goal);
};


#endif // PATHFINDING_H