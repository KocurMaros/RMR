#ifndef MAPPING_H
#define MAPPING_H

// #include "mainwindow.h"
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>

#include <rplidar.h>
#include <qmath.h>
#include <math.h>
#include "hash_map.h"
#include "point.h"
#include <queue>
#include <algorithm>


struct PointQueue {
    int x, y;
    int distance; // Distance from starting point

    bool operator<(const PointQueue& other) const {
    return distance > other.distance; // Prioritize closer points
    }
};
class Mapping
{
private:
    std::vector<Hash_map> map_vector;
    Hash_map map;
    std::string  map_file = "map_rework3.txt";
public:
    Mapping(/* args */);
    ~Mapping();
    void Gmapping(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta);
    void create_map(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta);
    Hash_map getMap(){return map;}
    void load_map();
    void save_map();
    void print_map();
    std::vector<Point> flood_fill(Point start, Point goal);
    std::vector<Point> floodFillPathfind(int startX, int startY, int goalX, int goalY);
    void create_new_map(double obstacleX, double obstacleY, double robotTheta);
};

#endif
