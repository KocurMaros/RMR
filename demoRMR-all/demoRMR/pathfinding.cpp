#include "pathfinding.h"

Pathfinding::Pathfinding(){
}

Pathfinding::~Pathfinding() {
}
void floodFillUtil(Hash_map map, int x, int y, int newC)
{
    // Base cases
    std::vector<std::vector<uint16_t>> hash_map = map.get_hash_map();
    int minX = map.get_boarder_minX();
    int maxX = map.get_boarder_maxX();
    int minY = map.get_boarder_minY();
    int maxY = map.get_boarder_maxY();
    if (x <= minX || x >= maxX || y <= minY || y >= maxY)
        return;
    if (hash_map[x][y] != 0) //visited or wall
        return;
    // Replace the color at (x, y)
    map.update_map(Point(x, y, 0), newC);
 
    // Recur for north, east, south and west
    floodFillUtil(map, x+1, y, newC+1);
    floodFillUtil(map, x-1, y, newC+1);
    floodFillUtil(map, x, y+1, newC+1);
    floodFillUtil(map, x, y-1, newC+1);
}
void Pathfinding::flood_fill(Mapping mapp, Point start, Point goal) {
    Hash_map map = mapp.getMap();
    // Flood fill implementation
    int x = start.getX();
    int y = start.getY();
    map.update_map(Point(goal.getX(),goal.getY(),goal.getTheta()), 2);
    // std::vector<std::vector<uint16_t>> hash_map = map.get_hash_map();
    // floodFillUtil(map, x, y, 3);
    // std::vector<std::vector<uint16_t>> hash_map2 = map.get_hash_map();
}
