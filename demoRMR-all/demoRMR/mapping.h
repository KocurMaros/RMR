#ifndef MAPPING_H
#define MAPPING_H

// #include "mainwindow.h"
#include <iomanip>
#include <rplidar.h>
#include <math.h>
#include "hash_map.h"
#include "point.h"

class Mapping
{
private:
    std::vector<Hash_map> map_vector;
    double minX = 100000, minY = 100000, maxX = -1000000, maxY = -1000000;
public:
    Mapping(/* args */);
    ~Mapping();
    void go_to_point(Point point);
    void Gmapping(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta);
    Hash_map create_map(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta);
    void merge_maps();
    void save_map();
};

#endif