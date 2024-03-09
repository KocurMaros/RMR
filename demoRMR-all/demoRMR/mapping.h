#ifndef MAPPING_H
#define MAPPING_H

#include "mainwindow.h"
#include "hash_map.h"
#include "point.h"

class Mapping
{
private:
    std::vector<std::vector<Hash_map>> map;
public:
    Mapping(/* args */);
    ~Mapping();
    void go_to_point(Point point);
    void Gmapping(LaserMeasurement laser_data, double robotX, double robotY);
};

#endif