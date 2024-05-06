#ifndef WALL_FOLLOWING_H
#define WALL_FOLLOWING_H
#include <rplidar.h>
#include "point.h"
#include "collision_detection.h"
#define WALL_FOLLOWING_VELOCITY 100.0 //mm/s
#define WALL_DISTANCE 0.3 //m
#define WALL_THRESHOLD 0.01 //m
#define ROBOT_L 0.23 // [m]


class WallFollowing {
public:
    WallFollowing();
    //set the laser data
    void setLaserData(LaserMeasurement laser_data) {this->laser_data = laser_data;}
    double getDesiredAnglePerpencidular() {return this->desired_angle_perpencidular;}
    bool isRotatedPerpendicularly() {return this->is_rotated_perpendicularly;}
    bool isNearWall() {return this->is_near_wall;}
    void setNearWall(bool is_near_wall) {this->is_near_wall = is_near_wall;}
    Point* getPoint()  {return &first_point;}
    void setPoint(Point point) {this->first_point = point;}
    void findWall(double robotFi, double robotX, double robotY);
    void computeDistancesToWall();
    double computeRotationVelocity();
    void checkIsRotatedPerpendicularly(double robotFi);
    void resetWallFollowing();
private:
    void computeDistanceLeft();
    void computeDistanceRight();
    void computeDistanceFront();
    double computeRotationVelocityRightSide();
    double computeRotationVelocityLeftSide();
    void setDistanceFlags();
    void calculateDesiredAngle();
    void computeFirstPoint(double robotFi, double robotX, double robotY);
    LaserMeasurement laser_data;
    Point first_point;
    double desired_angle_perpencidular;
    double lidar_min;
    double wall_angle;
    double wall_distance;
    double front_distance;
    double left_distance;
    double right_distance;
    double translation_velocity;
    double rotation_velocity;
    // wall by v tomto pripade mala byt najdena vzdy preto to nepouzivam
    // bool wall_found;
    bool is_rotated_perpendicularly;
    bool is_near_wall;
    //TODO: sharp angle...
    bool rotate_only;
    //TODO: both sides wall following
    bool follow_right_side;
    bool left;
    bool front;
    bool right;
};

#endif // WALL_FOLLOWING_H
