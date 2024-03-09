#ifndef HASH_MAP_H
#define HASH_MAP_H

#include <iostream>
#include <vector>
#include "point.h"

class Hash_map
{
private:
    uint8_t square_dimension = 10; //10x10cm square
    std::vector<std::vector<uint8_t>> hash_map; // 0 free 1 occupied
    std::vector<std::vector<Point>> coordinates;  // middle is 5 and 5
public:
    Hash_map(double x, double y);
    ~Hash_map();
    void update_map(Point point, bool occupied);
    std::vector<std::vector<uint8_t>> Hash_map::get_hash_map();
    std::vector<std::vector<Point>> get_coordinates();
};


#endif