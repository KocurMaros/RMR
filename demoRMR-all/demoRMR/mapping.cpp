#include "mapping.h"


Mapping::Mapping(/* args */)
{
}

Mapping::~Mapping()
{
}

void Mapping::go_to_point(Point point)
{
    //TODO
}

void Mapping::Gmapping(LaserMeasurement laser_data, double robotX, double robotY)
{
    
    Hash_map new_map = Hash_map(robotX, robotY);
    for(size_t i = 0; i < laser_data.numberOfScans; i++)
    {
        if(laser_data.Data[i].scanDistance > 5.6){ //5.6m is the maximum range of the laser
            continue;
        }
        double x = robotX + laser_data.Data[i].scanDistance * cos(laser_data.Data[i].scanAngle);
        double y = robotY + laser_data.Data[i].scanDistance * sin(laser_data.Data[i].scanAngle);
        Point point = Point(x, y, 0);
        new_map.update_map(point, true);
    }
    map_vector.push_back(new_map);
}