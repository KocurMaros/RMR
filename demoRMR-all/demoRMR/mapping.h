#ifndef MAPPING_H
#define MAPPING_H

// #include "mainwindow.h"
#include <iomanip>
#include <fstream>

#include <rplidar.h>
#include <math.h>
#include "hash_map.h"
#include "point.h"

class Mapping
{
private:
    bool geno = true;
    std::vector<Hash_map> map_vector;
    Hash_map map;
public:
    Mapping(/* args */);
    ~Mapping();
    void Gmapping(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta);
    void create_map(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta);
    void load_map();
    void save_map();
    void print_map();
};

#endif