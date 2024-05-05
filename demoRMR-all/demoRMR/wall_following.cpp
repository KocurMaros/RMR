#include "wall_following.h"

//robi sa v metroch, stupne
//follow right side znamena ze napravo od seba ma stenu
//TODO: na prvy bod treba prist cca 30 centi od steny i guess mozem vyuzit controller

WallFollowing::WallFollowing(){
    lidar_min = 10000;
    follow_right_side = false;
    left = false;
    front = false;
    right = false;
};

void WallFollowing::resetWallFollowing(){
    lidar_min = 10000;
    follow_right_side = false;
    left = false;
    front = false;
    right = false;
}

void WallFollowing::computeFirstPoint(double robotFi, double robotX, double robotY){
    double alfa = (wall_angle + robotFi)*PI/180;
    if (alfa >= PI) alfa -= 2*PI;
    else if (alfa < -PI) alfa += 2*PI;
    double c = wall_distance - 0.3;
    double x = robotX +  c * cos(alfa);
    double y = robotY + c * sin(alfa);
    first_point.setPoint(x*1000,y*1000,0);
}


void WallFollowing::findWall(double robotFi,double robotX, double robotY){
    //TODO:
    double lidar_angle;
    double lidar_distance;
    for(int k=0;k<laser_data.numberOfScans/*360*/;k++){
        lidar_angle = CollisionDetection::normalizeLidarAngle(laser_data.Data[k].scanAngle);
        lidar_distance = laser_data.Data[k].scanDistance/1000.0;
        if (lidar_distance <= lidar_min && lidar_distance != 0.0){
            lidar_min = laser_data.Data[k].scanDistance/1000.0;
            wall_angle = lidar_angle;
            wall_distance = lidar_distance;
        }
    }
    std::cout << "wall angle: " << wall_angle << std::endl;
    calculateDesiredAngle();
    is_rotated_perpendicularly = false;
    is_near_wall = false;
    computeFirstPoint(robotFi,robotX,robotY);
}

void WallFollowing::calculateDesiredAngle(){
    if (follow_right_side){
        desired_angle_perpencidular = wall_angle + 90;
    }else{
        desired_angle_perpencidular = wall_angle - 90;
    }
    if (desired_angle_perpencidular >= 180) desired_angle_perpencidular -= 360;
    else if (desired_angle_perpencidular < -180) desired_angle_perpencidular += 360;
    std::cout << "desired angle: " << desired_angle_perpencidular << std::endl;
}

void WallFollowing::checkIsRotatedPerpendicularly(double robotFi){
    if (abs(robotFi - desired_angle_perpencidular)<=4){
        is_rotated_perpendicularly = true;
    }
    else is_rotated_perpendicularly = false;
}
