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
double deg2rad(double deg)
{
    return deg * M_PI / 180;
}
double shift_theta(double theta)
{
    if(theta < 0){
        return -1.0 * theta;
    }
    else{
        return 360.0 - theta;
    }
}
// [0,0.1]
Hash_map Mapping::create_map(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta){
    Hash_map new_map = Hash_map(robotX, robotY, robotTheta);
    double angle, distance;
    double obstacle_x, obstacle_y;
    std::cout << shift_theta(robotTheta) << std::endl;
    uint16_t dim = new_map.get_map_dimension();
    dim = 300;
    for(size_t i = 0; i < laser_data.numberOfScans; i++)
    {
        if(laser_data.Data[i].scanDistance == 0){
            continue;
        }
        angle = shift_theta(robotTheta) + laser_data.Data[i].scanAngle ;
        distance = laser_data.Data[i].scanDistance/10.0;
        // std::cout << "angle: " << angle << " distance: " << distance << std::endl;
        obstacle_x = robotX -  distance * cos(deg2rad(angle));
        obstacle_y = robotY -  distance * sin(deg2rad(angle));
        // std::cout << "obstacle_x: " << obstacle_x << " obstacle_y: " << obstacle_y << std::endl;
        if(obstacle_x <= (robotX+dim) && obstacle_x >= (robotX-dim) && obstacle_y <= (robotY+dim) && obstacle_y >= (robotY-dim)){
            // std::cout << std::endl << std::endl;
            // std::cout << "obstacle_x: " << obstacle_x << " obstacle_y: " << obstacle_y << "blyyaat " << laser_data.Data[i].scanAngle <<  std::endl;
            if(obstacle_x < minX)
                minX = obstacle_x;
            if(obstacle_x > maxX)
                maxX = obstacle_x;
            if(obstacle_y < minY)
                minY = obstacle_y;
            if(obstacle_y > maxY)
                maxY = obstacle_y;
            Point point = Point(obstacle_x, obstacle_y, 0);
            new_map.update_map(point, true);
        }
    }
    return new_map;
}
Point new_position(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta){
    double angle, distance;
    double obstacle_x, obstacle_y;
    for(size_t i = 0; i < laser_data.numberOfScans; i++)
    {
        angle = shift_theta(robotTheta) + laser_data.Data[i].scanAngle ;
        distance = laser_data.Data[i].scanDistance/10.0;
        // std::cout << "angle: " << angle << " distance: " << distance << std::endl;
        obstacle_x = robotX +  distance * cos(deg2rad(angle));
        obstacle_y = robotY +  distance * sin(deg2rad(angle));
        // std::cout << "obstacle_x: " << obstacle_x << " obstacle_y: " << obstacle_y << std::endl;
        if(obstacle_x <= (robotX+50) && obstacle_x >= (robotX-50) && obstacle_y <= (robotY+50) && obstacle_y >= (robotY-50)){
            // std::cout << std::endl << std::endl;
            Point point = Point(obstacle_x, obstacle_y, 0);
            return point;
        }
    }
    return Point(0, 0, 0);
}
void Mapping::Gmapping(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta)
{
    Point next_position = Point(0, 0, 0);
    map_vector.push_back(create_map(laser_data, robotX, robotY, robotTheta));   //scan for obstacles and update map
    // next_position = new_position(laser_data, robotX, robotY, robotTheta);
    // std::cout << "Next position: " << next_position.getX() << " " << next_position.getY() << std::endl;
    // go_to_point(next_position);
    
    std::cout << std::fixed;
    std::cout << std::setprecision(1);

    for(size_t i = 0; i < map_vector.size(); i++){
        std::vector<std::vector<uint8_t>> map = map_vector[i].get_hash_map();
        std::vector<std::vector<Point>> coordinates = map_vector[i].get_coordinates();
        std::cout << minX << " " << minY << " " << maxX << " " << maxY << std::endl;
        std::cout << "________________________________________________________________________________________________________________________" << std::endl;
        for(size_t j = 0; j < map.size(); j++){
            std::cout << "|" << std::endl;
            for(size_t k = 0; k < map[j].size(); k++){
                if(map[j][k] == 1){
                    std::cout << "X ";
                }
                else{
                    std::cout << "  ";
                }
            }
            // std::cout << "   ";
            // for(size_t k = 0; k < coordinates[j].size(); k++){
            //     std::cout << "[" << coordinates[j][k].getX() << "," << coordinates[j][k].getY() << "]";
            // }
            std::cout << "|" << std::endl;
        }
    }
}

void Mapping::merge_maps()
{
    //TODO
}

void Mapping::save_map()
{
    //TODO
}