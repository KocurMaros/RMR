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
    double c = wall_distance - WALL_DISTANCE;
    double x = robotX +  c * cos(alfa);
    double y = robotY + c * sin(alfa);
    first_point.setPoint(x*1000,y*1000,0);
}

void WallFollowing::computeDistancesToWall(){
    double lidar_angle;
    double lidar_distance;
    int left_count = 0;
    int right_count = 0;
    int front_count = 0;
    for(int k=0;k<laser_data.numberOfScans/*360*/;k++){
        lidar_angle = CollisionDetection::normalizeLidarAngle(laser_data.Data[k].scanAngle);
        lidar_distance = laser_data.Data[k].scanDistance/1000.0;
        if (lidar_angle >= 87 && lidar_angle <= 92){
            //left
            if (lidar_distance != 0 ){
                left_count++;
                left_distance += lidar_distance;
            }
        }
        else if (lidar_angle <= -87 && lidar_angle <= -92){
            //right
            if (lidar_distance != 0){
                right_count++;
                right_distance += lidar_distance;
            }
        }
        else if((lidar_angle >=0 && lidar_angle <=2)||(lidar_angle <=0 && lidar_angle >= -2)){
            //front
            if (lidar_distance != 0){
                front_count++;
                front_distance += lidar_distance;
            }
        }
    }
    if (left_count != 0){
        left_distance = left_distance/left_count;
    }
    if (right_count != 0){
        right_distance = right_distance/right_count;
    }
    if (front_count != 0){
        front_distance = front_distance/front_count;
    }
    computeDistanceLeft();
    computeDistanceRight();
    computeDistanceFront();
}


void WallFollowing::computeDistanceLeft(){
    if(left_distance < (WALL_DISTANCE-WALL_THRESHOLD) && left_distance!=0){
        left = true;
    }
    else{
        left = false;
    }
}

void WallFollowing::computeDistanceRight(){
    if(right_distance < (WALL_DISTANCE-WALL_THRESHOLD) && right_distance!=0){
        right = true;
    }
    else{
        right = false;
    }
}

void WallFollowing::computeDistanceFront(){
    if(front_distance < (WALL_DISTANCE-WALL_THRESHOLD) && front_distance!=0){
        front = true;
    }
    else{
        front = false;
    }
}

double WallFollowing::computeRotationVelocity(){
    if (follow_right_side){
        return -computeRotationVelocityRightSide()/1000.0;
    }
    else {
        return -computeRotationVelocityLeftSide()/1000.0;
    }
}

double WallFollowing::computeRotationVelocityRightSide(){
    if(!front && !right){
        std::cout << "speed1 = " << -WALL_FOLLOWING_VELOCITY/WALL_DISTANCE << std::endl;
        return -WALL_FOLLOWING_VELOCITY/WALL_DISTANCE;
    }
    else if(front && !right){
        std::cout << "speed2 = " << 2*WALL_FOLLOWING_VELOCITY/ROBOT_L << std::endl;
        return 2*WALL_FOLLOWING_VELOCITY/ROBOT_L;
    }
    else if(!front && right){
        std::cout << "speed3 = " << WALL_FOLLOWING_VELOCITY/WALL_DISTANCE << std::endl;
        if(right_distance <= WALL_DISTANCE-WALL_THRESHOLD){
            return WALL_FOLLOWING_VELOCITY/WALL_DISTANCE;
        }
        else{
            return 0;
        }
    }
    else if(front && right){
        std::cout << "speed4 = " << 2*WALL_FOLLOWING_VELOCITY/ROBOT_L << std::endl;
        return 2*WALL_FOLLOWING_VELOCITY/ROBOT_L/1000.0;
    }
}

double WallFollowing::computeRotationVelocityLeftSide(){
    if(!front && !left){
        std::cout << "speed1 = " << -WALL_FOLLOWING_VELOCITY/WALL_DISTANCE << std::endl;
        return WALL_FOLLOWING_VELOCITY/WALL_DISTANCE;
    }
    else if(front && !left){
        std::cout << "speed2 = " << -2*WALL_FOLLOWING_VELOCITY/ROBOT_L << std::endl;
        return -2*WALL_FOLLOWING_VELOCITY/ROBOT_L;
    }
    else if(!front && left){
        std::cout << "speed3 = " << -WALL_FOLLOWING_VELOCITY/WALL_DISTANCE << std::endl;
        if(left_distance <= WALL_DISTANCE-WALL_THRESHOLD){
            return -WALL_FOLLOWING_VELOCITY/WALL_DISTANCE;
        }
        else{
            return 0;
        }
    }
    else if(front && left){
        std::cout << "speed4 = " << -2*WALL_FOLLOWING_VELOCITY/ROBOT_L << std::endl;
        return -2*WALL_FOLLOWING_VELOCITY/ROBOT_L;
    }
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
    //
    std::cout << "wall angle: " << wall_angle << std::endl;
    if (wall_angle <= 180 && wall_angle >= 0){
        follow_right_side = true;
    }
    else{
        follow_right_side = false;
    }
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
