#include "mapping.h"


Mapping::Mapping(/* args */)
{
}

Mapping::~Mapping()
{
}

double deg2rad(double deg)
{
    return deg * M_PI / 180.0;
}
double shift_theta(double theta)
{
    return 360.0 - theta;
}
/*
[180 0 0 -180]
*/
double shift_theta_robot(double theta)
{
    if(theta < 0){
        return -1.0 * theta;
    }
    else{
        return 360.0 - theta;
    }
}
// [0,0.1]
void Mapping::create_map(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta){
    int bad = 0;
    for(size_t u = 0; u < map_vector.size(); u++){
        double angle, distance;
        double obstacle_x, obstacle_y;
        // std::cout << shift_theta(robotTheta) << std::endl;
        uint16_t dim = map_vector[u].get_map_dimension();
        dim = 300;
        for(size_t i = 0; i < laser_data.numberOfScans; i++)
        {
            if(laser_data.Data[i].scanDistance == 0 || laser_data.Data[i].scanDistance > 3200){
                continue;
            }
            // angle = shift_theta(robotTheta) + laser_data.Data[i].scanAngle ;

            angle = shift_theta(shift_theta_robot(robotTheta) + laser_data.Data[i].scanAngle);
            distance = laser_data.Data[i].scanDistance/10.0;
            obstacle_x = robotX +  distance * cos(deg2rad(angle));
            obstacle_y = robotY +  distance * sin(deg2rad(angle));
            if(obstacle_x < map_vector[u].get_minX() || obstacle_x > map_vector[u].get_maxX() || obstacle_y < map_vector[u].get_minY() || obstacle_y > map_vector[u].get_maxY()){
                // std::cout << map_vector[u].get_minX() << " " << map_vector[u].get_minY() << " " << map_vector[u].get_maxX() << " " << map_vector[u].get_maxY() << std::endl;
                // std::cout << obstacle_x << " " << obstacle_y << std::endl;
                // std::cout << laser_data.Data[i].scanDistance << " " << laser_data.Data[i].scanAngle << " " << shift_theta_robot(robotTheta) << " " << angle << std::endl; 
                // std::cout << map_vector[u].get_minX() << " " << map_vector[u].get_minY() << " " << map_vector[u].get_maxX() << " " << map_vector[u].get_maxY() << std::endl;
                bad ++;
                continue;
            }
            // if(obstacle_x <= (robotX+dim) && obstacle_x >= (robotX-dim) && obstacle_y <= (robotY+dim) && obstacle_y >= (robotY-dim)){
            Point point = Point(obstacle_x, obstacle_y, 0);
            // std::cout << obstacle_x << " " << obstacle_y << " " << u << std::endl;
            map_vector[u].update_map(point, true);
            // }
        }
        // std::cout << "bad " << bad << std::endl; 
        bad = 0;
    }
}

void Mapping::Gmapping(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta)
{
    // std::cout << "robotX "   << robotX << " robotY " << robotY << " robotTheta " << robotTheta << std::endl;  
    // Point next_position = Point(0, 0, 0);
    if(geno){
        geno = false;
        map_vector.push_back(Hash_map(0, 0, robotTheta, 10, 63));
        map_vector.push_back(Hash_map(620.0, 0, robotTheta, 10, 63)); 
        map_vector.push_back(Hash_map(0, 620.0, robotTheta, 10, 63));    //scan for obstacles and update map
        map_vector.push_back(Hash_map(620.0, 620.0, robotTheta, 10, 63));    //scan for obstacles and update map
    }
    
    create_map(laser_data, robotX*100., robotY*100., robotTheta);
}

void Mapping::load_map()
{
    std::string line;
    std::ifstream MyFile("map_file3.txt");
    double centerX, centerY, width;
    if (getline(MyFile, line)) {
        std::istringstream iss(line);
        if (!(iss >> centerX >> centerY >> width)) {
            std::cout << "Error reading" << std::endl;
        }
        std::cout << "num1: " << centerX << ", num2: " << centerY << ", num3: " << width << std::endl;
    }
    map = Hash_map(centerX, centerY, 0, 10, width);

    while (getline(MyFile, line)) {
        size_t start = line.find('[');
        size_t end = line.find(']');
        while (start != std::string::npos && end != std::string::npos) {
            std::string coordinates = line.substr(start + 1, end - start - 1);
            std::istringstream iss(coordinates);
            int arg1, arg2;
            char comma;
            if (iss >> arg1 >> comma >> arg2) {
                // std::cout << arg1 << " " << arg2 << std::endl;
                Point point(arg1, arg2, 0);
                map.update_map(point, 1);
            }
            start = line.find('[', end);
            end = line.find(']', start);
        }
    }
    MyFile.close();
    // std::vector<std::vector<uint8_t>> map1 = map.get_hash_map();
    // std::vector<std::vector<Point>> coordinates = map.get_coordinates();
    // std::cout << "________________________________________________________________________________________________________________________" << std::endl;
    // for(size_t j = 0; j < map1.size(); j++){
    //     std::cout << "|";
    //     for(size_t k = 0; k < map1[j].size(); k++){
    //         if(map1[j][k] == 1){
    //             std::cout << "X ";
    //         }
    //         else{
    //             std::cout << "  ";
    //         }
    //     }
    //     // std::cout << "|  ";
    //     // for(size_t k = 0; k < coordinates[j].size(); k++){
    //     //     std::cout << "[" << coordinates[j][k].getX() << "," << coordinates[j][k].getY() << "]";
    //     // }
    //     std::cout << "|" << std::endl;
    // }
}

void Mapping::save_map()
{
    double minX = map_vector[0].get_boarder_minX(), minY = map_vector[0].get_boarder_minY(), maxX = map_vector[0].get_boarder_maxX(), maxY = map_vector[0].get_boarder_maxY();
    for(size_t i = 1; i < map_vector.size(); i++){
        if(minX > map_vector[i].get_boarder_minX())
            minX = map_vector[i].get_boarder_minX();
        if(minY > map_vector[i].get_boarder_minY())
            minY = map_vector[i].get_boarder_minY();
        if(maxX < map_vector[i].get_boarder_maxX())
            maxX = map_vector[i].get_boarder_maxX();
        if(maxY < map_vector[i].get_boarder_maxY())
            maxY = map_vector[i].get_boarder_maxY();
    }
    double centerX, centerY, width;
    centerX = (maxX + minX)/2.0;
    centerY = (maxY + minY)/2.0;
    width = maxX - minX;
    if(width < maxY - minY){
        width = maxY - minY;
    }
    std::cout << centerX << " " << centerY << " " << width << std::endl;
    Hash_map whole_map = Hash_map(centerX, centerY, 0, 10, width);
    for(size_t i = 0; i < map_vector.size(); i++){
        std::vector<std::vector<uint8_t>> map = map_vector[i].get_hash_map();
        std::vector<std::vector<Point>> coordinates = map_vector[i].get_coordinates();
        for(size_t j = 0; j < map.size(); j++){
            for(size_t k = 0; k < map[j].size(); k++){
                if(map[j][k] == 1){
                    Point point = Point(coordinates[j][k].getX(), coordinates[j][k].getY(), 0);
                    whole_map.update_map(point, true);
                }
            }
        }
    }
    std::ofstream MyFile("map_file3.txt");

    MyFile << centerX << " " << centerY << " " << width << std::endl;
    std::vector<std::vector<uint8_t>> map1 = whole_map.get_hash_map();
    std::vector<std::vector<Point>> coordinates = whole_map.get_coordinates();
    bool was_one = false;
    for(size_t i = 0; i < map1.size(); i++){
        for(size_t j = 0; j < map1[i].size(); j++){
            if(map1[i][j] == 1){
                was_one = true;
                // MyFile << "1";
                MyFile << "["<< coordinates[i][j].getX()<< "," << coordinates[i][j].getY() << "]";
            }
            // else{
            //     MyFile << "0";
            // }
        }
        if(was_one){
            MyFile << std::endl;
            was_one = false;
        }
    }
    MyFile.close();
}
void Mapping::print_map(){
    std::vector<std::vector<uint8_t>> map1 = map.get_hash_map();
    std::vector<std::vector<Point>> coordinates = map.get_coordinates();
    for(size_t j = 0; j < map1.size(); j++){
        std::cout << "|";
        for(size_t k = 0; k < map1[j].size(); k++){
            if(map1[j][k] == 1){
                std::cout << "X ";
            }
            else{
                std::cout << "  ";
            }
        }
        std::cout << "|  ";
        for(size_t k = 0; k < coordinates[j].size(); k++){
            std::cout << "[" << coordinates[j][k].getX() << "," << coordinates[j][k].getY() << "]";
        }
        std::cout << "|" << std::endl;
    }
}