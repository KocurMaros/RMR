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
    double getDistance() const {return distance;}
    double getAngle() const {return angle;}
    bool isFoundEdge() const {return found_edge;}
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
    double getDistance() const {return distance;}
    double getAngle() const {return angle;}
    int getIndex() const {return index;}
    void setDistance(double distance) {this->distance = distance;}
    void setAngle(double angle) {this->angle = angle;}
    void setIndex(int index) {this->index = index;}
    Edge getLeftEdge() const {return left_edge;}
    Edge getRightEdge() const {return right_edge;}
    bool isFoundObstacle() const {return found_obstacle;}
    void setFoundObstacle(bool found_obstacle) {this->found_obstacle = found_obstacle;}
    //TODO: scalovat podla initial vzdialenosti objektu (alebo podla vzdialenosti predosleho bodu ci co)?
    //TODO: doladit velkost distanceThresholdu.....
    constexpr static const double distanceThreshold = 0.4;
};

class CollisionDetection {
public:
    bool isObstacleInPath(double scanDistance, double scanAngle, double zoneAngle, double zoneDistance);
    Obstacle getObstacle() const {return obstacle;}
    static double normalizeLidarAngle(double angle);
    void setLaserData(LaserMeasurement laser_data) {this->laser_data = laser_data;}
    LaserMeasurement getLaserData() const {return laser_data;}

private:
    Obstacle obstacle;
    LaserMeasurement laser_data;
};

#endif // COLLISION_DETECTION_H
