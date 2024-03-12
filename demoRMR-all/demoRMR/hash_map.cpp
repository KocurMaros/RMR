#include "hash_map.h"

Hash_map::Hash_map(double x, double y, double theta)
{
    for (uint8_t i = 0; i < 11; i++)
    {
        std::vector<Point> row;
        std::vector<uint8_t> row_hash;
        for (uint8_t j = 0; j < 11; j++)
        {
            row_hash.push_back(0);
            row.push_back(Point(x + (i - 5) * square_dimension, y + (j - 5) * square_dimension, 0));
        }
        hash_map.push_back(row_hash);
        coordinates.push_back(row);
    }
}
Hash_map::~Hash_map(){}

void Hash_map::update_map(Point point, bool occupied){
    int x = point.getX();
    int y = point.getY();
    int i = 0;
    int j = 0;
    for (i = 0; i < 11; i++)
    {
        if (coordinates[i][0].getX() < x && coordinates[i + 1][0].getX() > x)
        {
            break;
        }
    }
    for (j = 0; j < 11; j++)
    {
        if (coordinates[0][j].getY() < y && coordinates[0][j + 1].getY() > y)
        {
            break;
        }
    }
    if(i < 11 && j < 11){
        if (occupied)
        {
            hash_map[i][j] = 1;
        }
        else
        {
            hash_map[i][j] = 0;
        }
    }

}


std::vector<std::vector<uint8_t>> Hash_map::get_hash_map(){
    return hash_map;
}
std::vector<std::vector<Point>> Hash_map::get_coordinates(){
    return coordinates;
}