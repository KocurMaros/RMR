#ifndef HASH_MAP_H
#define HASH_MAP_H

#include <iostream>
#include <vector>
#include "point.h"

class Hash_map
{
private:
    uint8_t square_dimension; //10x10cm square
    uint8_t map_dimension; // 60x60 squares
    std::vector<std::vector<uint8_t>> hash_map; // 0 free 1 occupied
    std::vector<std::vector<Point>> coordinates;  // middle is 5 and 5
    double minX = 100000, minY = 100000, maxX = -1000000, maxY = -1000000;
    double boarder_minX = 100000, boarder_minY = 100000, boarder_maxX = -1000000, boarder_maxY = -1000000;

public:
    Hash_map() = default;
    Hash_map(double x, double y, double theta, uint8_t square_dim, uint8_t map_dim);
    ~Hash_map();
    void update_map(Point point, bool occupied);
    std::vector<std::vector<uint8_t>> get_hash_map();
    std::vector<std::vector<Point>> get_coordinates();
    double get_minX(){return minX;}
    double get_minY(){return minY;}
    double get_maxX(){return maxX;}
    double get_maxY(){return maxY;}
    double get_boarder_minX(){return boarder_minX;}
    double get_boarder_minY(){return boarder_minY;}
    double get_boarder_maxX(){return boarder_maxX;}
    double get_boarder_maxY(){return boarder_maxY;}
    uint8_t get_map_dimension(){return map_dimension;}
};


#endif