#ifndef WALL_FOLLOWING_H
#define WALL_FOLLOWING_H
#include <rplidar.h>
#include "collision_detection.h"
#define WALL_FOLLOWING_VELOCITY 300 //mm/s
#define WALL_DISTANCE 0.1 //m

class WallFollowing {
public:
    WallFollowing();
    //set the laser data
    void setLaserData(LaserMeasurement laser_data) {this->laser_data = laser_data;}
    double getDesiredAnglePerpencidular() {return this->desired_angle_perpencidular;}
    bool isRotatedPerpendicularly() {return this->is_rotated_perpendicularly;}
    void findWall(double robotFi);
    void computeDistancesToWall();
    void computeVelocities();
    void computeRotationVelocity();
    void checkIsRotatedPerpendicularly(double robotFi);
    void resetWallFollowing();
private:
    void computeDistanceLeft();
    void computeDistanceRight();
    void computeDistanceFront();
    void setDistanceFlags();
    void calculateDesiredAngle();
    LaserMeasurement laser_data;
    double desired_angle_perpencidular;
    double lidar_min;
    double wall_angle;
    double front_distance;
    double left_distance;
    double right_distance;
    double translation_velocity;
    double rotation_velocity;
    // wall by v tomto pripade mala byt najdena vzdy preto to nepouzivam
    // bool wall_found;
    bool is_rotated_perpendicularly;
    //TODO: sharp angle...
    bool rotate_only;
    //TODO: both sides wall following
    bool follow_right_side;
    bool left;
    bool front;
    bool right;
};

#endif // WALL_FOLLOWING_H
