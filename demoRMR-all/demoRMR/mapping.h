#ifndef MAPPING_H
#define MAPPING_H

#include "mainwindow.h"
#include "hash_map.h"
#include "point.h"

class Mapping
{
private:
    // std::vector<std::vector<Hash_map>> map_vector;
    std::vector<Hash_map> map_vector;
    double map_resolution = 10; //10x10cm square
    double map_size = 55; //10x10cm square
public:
    Mapping(/* args */);
    ~Mapping();
    void go_to_point(Point point);
    void Gmapping(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta);
};

#endif