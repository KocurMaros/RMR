#include "hash_map.h"

Hash_map::Hash_map(double x, double y, double theta, uint8_t square_dim, uint8_t map_dim)
{
    square_dimension = square_dim;
    map_dimension = map_dim;
    double xx;
    double yy;
    for (uint8_t i = 0; i < map_dimension; i++)
    {
        std::vector<Point> row;
        std::vector<uint16_t> row_hash;
        for (uint8_t j = 0; j < map_dimension; j++)
        {
            row_hash.push_back(0);
            xx = x + (i - (map_dimension-1)/2) * square_dimension;
            yy = y + (j - (map_dimension-1)/2) * square_dimension;
            row.push_back(Point(xx, yy, 0,i,j));

            if(xx < minX)
                minX = xx;
            if(xx > maxX)
                maxX = xx;
            if(yy < minY)
                minY = yy;
            if(yy > maxY)
                maxY = yy;
        }
        hash_map.push_back(row_hash);
        coordinates.push_back(row);
    }
}
Hash_map::~Hash_map(){}

void Hash_map::update_map(Point point, uint16_t occupied){

    int index_x, index_y;

    double min_x = coordinates[0][0].getX();
    double min_y = coordinates[0][0].getY();
    double dx = coordinates[1][0].getX() - min_x;
    double dy = coordinates[0][1].getY() - min_y;
    index_x = static_cast<int>((point.getX() - min_x) / dx);
    index_y = static_cast<int>((point.getY() - min_y) / dy);

    if(index_x < map_dimension && index_y < map_dimension){
        if(coordinates[index_x][0].getX() < boarder_minX)
            boarder_minX = coordinates[index_x][0].getX();
        if(coordinates[index_x][0].getX() > boarder_maxX)
            boarder_maxX = coordinates[index_x][0].getX();
        if(coordinates[0][index_y].getY() < boarder_minY)
            boarder_minY = coordinates[0][index_y].getY();
        if(coordinates[0][index_y].getY() > boarder_maxY)
            boarder_maxY = coordinates[0][index_y].getY();
        hash_map[index_x][index_y] = occupied;
    }
}


std::vector<std::vector<uint16_t>> Hash_map::get_hash_map(){
    return hash_map;
}
std::vector<std::vector<Point>> Hash_map::get_coordinates(){
    return coordinates;
}