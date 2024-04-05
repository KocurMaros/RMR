#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H
#include <math.h>
#include <rplidar.h>


class Edge{
private:
    double distance;
    double angle;
    bool found_edge;
public:
    void setDistance(double distance) {this->distance = distance;}
    void setAngle(double angle) {this->angle = angle;}
    double getDistance()  {return distance;}
    double getAngle()  {return angle;}
    bool isFoundEdge()  {return found_edge;}
    void setFoundEdge(bool found_edge) {this->found_edge = found_edge;}
};

class Obstacle{
private:
    bool found_obstacle;
    double distance;
    double angle;
    int index;
    Edge left_edge;
    Edge right_edge;

public:
    double getDistance()  {return distance;}
    double getAngle()  {return angle;}
    int getIndex()  {return index;}
    void setDistance(double distance) {this->distance = distance;}
    void setAngle(double angle) {this->angle = angle;}
    void setIndex(int index) {this->index = index;}
    Edge* getLeftEdge()  {return &left_edge;}
    Edge* getRightEdge()  {return &right_edge;}
    bool isFoundObstacle()  {return found_obstacle;}
    void setFoundObstacle(bool found_obstacle) {this->found_obstacle = found_obstacle;}
    //TODO: scalovat podla initial vzdialenosti objektu (alebo podla vzdialenosti predosleho bodu ci co)?
    //TODO: doladit velkost distanceThresholdu.....
    constexpr static const double distanceThreshold = 0.4;
};

class CollisionDetection {
public:
    bool isObstacleInPath(double scanDistance, double scanAngle, double zoneAngle, double zoneDistance);
    Obstacle *getObstacle()  {return &obstacle;}
    static double normalizeLidarAngle(double angle);
    void setLaserData(LaserMeasurement laser_data) {this->laser_data = laser_data;}
    LaserMeasurement getLaserData()  {return laser_data;}

private:
    Obstacle obstacle;
    LaserMeasurement laser_data;
};

#endif // COLLISION_DETECTION_H
