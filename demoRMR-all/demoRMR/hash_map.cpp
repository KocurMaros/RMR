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
            // std::cout << "x " << x + (i - (map_dimension-1)/2) * square_dimension << " y " << y + (j - (map_dimension-1)/2) * square_dimension << std::endl;
        }
        hash_map.push_back(row_hash);
        coordinates.push_back(row);
    }
    // std::cout << minX << " " << maxX << " " << minY << " " << maxY << std::endl;
    // std::cout << "x " << x << " y " << y << " theta " << theta << " square_dim " << square_dimension << " map_dim " << map_dimension << std::endl;
}
Hash_map::~Hash_map(){}

void Hash_map::update_map(Point point, uint16_t occupied){
    int x = point.getX();
    int y = point.getY();
    int i = 0;
    int j = 0;

    for (i = 0; i < map_dimension; i++)
    {
        if (coordinates[i][0].getX() <= x && coordinates[i + 1][0].getX() > x)
        {
            break;
        }
    }
    for (j = 0; j < map_dimension; j++)
    {
        if (coordinates[0][j].getY() <= y && coordinates[0][j + 1].getY() > y)
        {
            break;
        }
    }
    //odratat min podelit square dim a zaokrouhlit na dolu
    if(i < map_dimension && j < map_dimension){
        if(coordinates[i][0].getX() < boarder_minX)
            boarder_minX = coordinates[i][0].getX();
        if(coordinates[i][0].getX() > boarder_maxX)
            boarder_maxX = coordinates[i][0].getX();
        if(coordinates[0][j].getY() < boarder_minY)
            boarder_minY = coordinates[0][j].getY();
        if(coordinates[0][j].getY() > boarder_maxY)
            boarder_maxY = coordinates[0][j].getY();
        hash_map[i][j] = occupied;
    }
}


std::vector<std::vector<uint16_t>> Hash_map::get_hash_map(){
    return hash_map;
}
std::vector<std::vector<Point>> Hash_map::get_coordinates(){
    return coordinates;
}