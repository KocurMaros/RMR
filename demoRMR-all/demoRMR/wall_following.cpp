#include "wall_following.h"

//robi sa v metroch, stupne
//follow right side znamena ze napravo od seba ma stenu


WallFollowing::WallFollowing(){
    lidar_min = 10000;
    follow_right_side = false;
    left = false;
    front = false;
    right = false;
};

void WallFollowing::findWall(double robotFi){
    //TODO:
    double lidar_angle;
    for(int k=0;k<laser_data.numberOfScans/*360*/;k++){
        lidar_angle = CollisionDetection::normalizeLidarAngle(laser_data.Data[k].scanAngle);
        if (laser_data.Data[k].scanDistance/1000.0 <= lidar_min){
            lidar_min = laser_data.Data[k].scanDistance/1000.0;
            wall_angle = lidar_angle;
        }
    }
    calculateDesiredAngle();
    is_rotated_perpendicularly = false;
    checkIsRotatedPerpendicularly(robotFi);
}

void WallFollowing::calculateDesiredAngle(){
    if (follow_right_side){
        desired_angle_perpencidular = wall_angle + 90;
    }else{
        desired_angle_perpencidular = wall_angle - 90;
    }
    if (desired_angle_perpencidular >= 180) desired_angle_perpencidular -= 360;
    else if (desired_angle_perpencidular < -180) desired_angle_perpencidular += 360;
}

void WallFollowing::checkIsRotatedPerpendicularly(double robotFi){
    if (abs(robotFi - desired_angle_perpencidular)<=4){
        is_rotated_perpendicularly = true;
    }
    else is_rotated_perpendicularly = false;
}
