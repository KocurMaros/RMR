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
void Mapping::create_new_map(double obstacleX, double obstacleY, double robotTheta){
    bool x_was = false, y_was = false; 
    int k = -10, l = -10;
    while(1){
        if(obstacleX < (k*620.0)+310.0 && obstacleX > (k*310.0)-310.0)
            break;
        k++;
    }
    while(1){
        if(obstacleY > (l*620.0)-310.0 && obstacleY < (l*620.0)+310.0)
            break;
        l++;
    }
    // k = (int)obstacleX/620;
    // l = (int)obstacleY/620;
    map_vector.push_back(Hash_map(620.0*(k), 620.0*l, robotTheta, 10, 63)); 
    std::cout << "Map created " << " " << 620.0*(k) << " " << 620.0*l << " " << robotTheta << std::endl;
}
void Mapping::create_map(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta){
    double angle, distance;
    double obstacle_x, obstacle_y;
    for(size_t i = 0; i < laser_data.numberOfScans; i++){
        if(laser_data.Data[i].scanDistance <= 650 || laser_data.Data[i].scanDistance > 3000)
            continue;
        angle = shift_theta(shift_theta_robot(robotTheta) + laser_data.Data[i].scanAngle);
        distance = laser_data.Data[i].scanDistance/10.0;
        obstacle_x = robotX +  distance * cos(deg2rad(angle));
        obstacle_y = robotY +  distance * sin(deg2rad(angle));
        for(size_t u = 0; u < map_vector.size(); u++){
            if(obstacle_x >= map_vector[u].get_minX() && obstacle_x <= map_vector[u].get_maxX() && obstacle_y >= map_vector[u].get_minY() && obstacle_y <= map_vector[u].get_maxY()){
                Point point = Point(obstacle_x, obstacle_y, 0);
                map_vector[u].update_map(point, true);
                break;
                // u = map_vector.size();
            }else if(u == map_vector.size()-1){
                std::cout << "Map creating " << " " << obstacle_x << " " << obstacle_y << " " << robotTheta << std::endl;
                create_new_map(obstacle_x, obstacle_y,robotTheta);
            }
        }
    }
}

void Mapping::Gmapping(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta)
{
    if(map_vector.empty())
        map_vector.push_back(Hash_map(0, 0, robotTheta, 10, 63));
    create_map(laser_data, robotX*100., robotY*100., robotTheta);
}

void Mapping::load_map()
{
    std::string line;
    std::ifstream MyFile(map_file);
    int centerX, centerY, width;
    if (getline(MyFile, line)) {
        std::istringstream iss(line);
        if (!(iss >> centerX >> centerY >> width)) {
            std::cout << "Error reading" << std::endl;
        }
        std::cout << "num1: " << centerX << ", num2: " << centerY << ", num3: " << width << std::endl;
    }
    if(centerX%2 != 0)
        centerX = centerX + 5;
    
    if(centerY%2 != 0)
        centerY = centerY +5;
    map = Hash_map(centerX, centerY, 0, 10, width);
    int line_ind=0 , col_ind = 0;
    while (getline(MyFile, line)) {
        size_t start = line.find('[');
        size_t end = line.find(']');
        while (start != std::string::npos && end != std::string::npos) {
            std::string coordinates = line.substr(start + 1, end - start - 1);
            std::istringstream iss(coordinates);
            int arg1, arg2;
            char comma;
            if (iss >> arg1 >> comma >> arg2) {
                map.update_map(Point(arg1,arg2,0), 1);
                for (int dx = -20; dx <= 20; dx+=10)
                    for(int dy = -20; dy <= 20; dy+=10)
                        map.update_map(Point(arg1+dx,arg2+dy,0), 1);
            }
            start = line.find('[', end);
            end = line.find(']', start);
            col_ind++;
        }
        line_ind++;
    }
    MyFile.close();
    std::cout << "load " << map.get_boarder_maxY() << " " << map.get_boarder_minY() << " " << map.get_boarder_maxX() << " " << map.get_boarder_minX() << std::endl;
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
    width = width/2.0;
    std::cout << centerX << " " << centerY << " " << width << std::endl;
    Hash_map whole_map = Hash_map(centerX, centerY, 0, 10, width);
    for(size_t i = 0; i < map_vector.size(); i++){
        std::vector<std::vector<uint16_t>> map = map_vector[i].get_hash_map();
        std::vector<std::vector<Point>> coordinates = map_vector[i].get_coordinates();
        for(size_t j = 0; j < map.size(); j++){
            for(size_t k = 0; k < map[j].size(); k++){
                if(map[j][k] == 1){
                    // std::cout << "map[j][k] " << j << " " << k << std::endl;
                    Point point = Point(coordinates[j][k].getX(), coordinates[j][k].getY(), 0);
                    whole_map.update_map(point, true);
                }
            }
        }
    }
    std::cout << "whole_map " << whole_map.get_boarder_maxY() << " " << whole_map.get_boarder_minY() << " " << whole_map.get_boarder_maxX() << " " << whole_map.get_boarder_minX() << std::endl;
    std::ofstream MyFile(map_file);

    MyFile << centerX << " " << centerY << " " << width << std::endl;
    std::vector<std::vector<uint16_t>> map1 = whole_map.get_hash_map();
    std::vector<std::vector<Point>> coordinates = whole_map.get_coordinates();
    bool was_one = false;
    for(size_t i = 0; i < map1.size(); i++){
        for(size_t j = 0; j < map1[i].size(); j++){
            if(map1[i][j] == 1){
                was_one = true;
                MyFile << "["<< coordinates[i][j].getX()<< "," << coordinates[i][j].getY() << "]";
            }
        }
        if(was_one){
            MyFile << std::endl;
            was_one = false;
        }
    }
    MyFile.close();
}
void Mapping::print_map(){
    std::vector<std::vector<uint16_t>> map1 = map.get_hash_map();
    std::vector<std::vector<Point>> coordinates = map.get_coordinates();
    for(size_t j = 0; j < map1.size(); j++){
        // std::cout << "|";
        for(size_t k = 0; k < map1[j].size(); k++){
            if(coordinates[j][k].getX() == 0 && coordinates[j][k].getY() == 0){
                std::cout << "X" << "   ";
            }else{
                if(map1[j][k] >= 100)
                    std::cout << map1[j][k] << " ";
                else if(map1[j][k] >= 10)
                    std::cout << map1[j][k] << "  ";
                else
                    std::cout << map1[j][k] << "   ";
            }
        }
        // std::cout << "|  ";
        // for(size_t k = 0; k < coordinates[j].size(); k++){
        //     std::cout << "[" << coordinates[j][k].getX() << "," << coordinates[j][k].getY() << "]";
        //     std::cout << map.get_coordinates()[j][k].getX() << " " <<map.get_coordinates()[j][k].getY(); 
        // }
        std::cout << "|" << std::endl;
    }
}

std::vector<Point> Mapping::flood_fill(Point start, Point goal) {

    int ind_start_x, ind_start_y;
    int ind_goal_x, ind_goal_y;   
    
    map.update_map(Point(goal.getX(),goal.getY(),goal.getTheta()), 2);

    int x = goal.getX();
    int y = goal.getY();

    double min_x = map.get_coordinates()[0][0].getX();
    double min_y = map.get_coordinates()[0][0].getY();
    double dxxx = map.get_coordinates()[1][0].getX() - min_x;
    double dyyy = map.get_coordinates()[0][1].getY() - min_y;

    // dakedy boli tu fory ne 
    ind_start_x = static_cast<int>((start.getX() - min_x) / dxxx);
    ind_start_y = static_cast<int>((start.getY() - min_y) / dyyy);
    ind_goal_x = static_cast<int>((goal.getX() - min_x) / dxxx);
    ind_goal_y = static_cast<int>((goal.getY() - min_y) / dyyy);

    std::cout << "Point coord" << start.getX() << " " << start.getY() << " " << goal.getX() << " " << goal.getY() << std::endl;
    std::cout << "ind_start_x " << ind_start_x << " ind_start_y " << ind_start_y << " ind_goal_x " << ind_goal_x << " ind_goal_y " << ind_goal_y << std::endl;
    std::cout << "Map ind " << map.get_coordinates()[ind_start_x][ind_start_y].getX() << " " << map.get_coordinates()[ind_start_x][ind_start_y].getY() << " " << map.get_coordinates()[ind_goal_x][ind_goal_y].getX() << " " << map.get_coordinates()[ind_goal_x][ind_goal_y].getY() << std::endl; 
    
    std::vector<Point> path = floodFillPathfind(ind_start_x, ind_start_y, ind_goal_x, ind_goal_y);

    std::vector<Point> filteredPath;

    int prev_shiftX = 0, prev_shiftY = 0;

    bool sameX = false, sameY = false;
    for(size_t i = 0; i < path.size(); i++){
        if(path[i].getX() == path[i+1].getX() && !sameY)
            sameX = true;
        if(path[i].getY() == path[i+1].getY() && !sameX)
            sameY = true;
        if(path[i].getX() == path[i+1].getX() && path[i].getY() != path[i+1].getY() && sameY){
            filteredPath.push_back(Point(path[i].getX(),path[i].getY(),0));
            sameY = false;
            continue;
        }
        if(path[i].getY() == path[i+1].getY() && path[i].getX() != path[i+1].getX() && sameX){
            filteredPath.push_back(Point(path[i].getX(),path[i].getY(),0));
            sameX = false;
            continue;
        }
    }
    filteredPath.push_back(Point(goal.getX()*10,goal.getY()*10,0));
    return filteredPath;
}

std::vector<Point> Mapping::floodFillPathfind(int startX, int startY, int goalX, int goalY) {
    std::vector<std::vector<uint16_t>> grid = map.get_hash_map();

    int next_index = 3;
    
    int rows = map.get_map_dimension();
    int cols = map.get_map_dimension();


    std::queue<Point> frontier;

    Point start = Point(goalX, goalY, 0);
    frontier.push(start);
    while (!frontier.empty()) {
        
        Point current = frontier.front();
        frontier.pop();

        if (current.getX() == startX && current.getY() == startY) {
            // print_map();
            uint16_t act_index;
            int x_path = current.getX();
            int y_path = current.getY();
            grid = map.get_hash_map();
            act_index = grid[x_path][y_path];
            std::vector<Point> path;
            path.push_back(Point(map.get_coordinates()[x_path][y_path].getX()*10, map.get_coordinates()[x_path][y_path].getY()*10, 0));

            while(act_index != 3){
                if(act_index > grid[x_path][y_path+1] && grid[x_path][y_path+1] > 1){
                    y_path = y_path + 1;
                    path.push_back(Point(map.get_coordinates()[x_path][y_path].getX()*10, map.get_coordinates()[x_path][y_path].getY()*10, 0));
                }else if(act_index > grid[x_path+1][y_path] && grid[x_path+1][y_path] > 1){
                    x_path = x_path + 1;
                    path.push_back(Point(map.get_coordinates()[x_path][y_path].getX()*10, map.get_coordinates()[x_path][y_path].getY()*10, 0));
                }else if(act_index > grid[x_path][y_path-1] && grid[x_path][y_path-1] > 1){
                    y_path = y_path - 1;
                    path.push_back(Point(map.get_coordinates()[x_path][y_path].getX()*10, map.get_coordinates()[x_path][y_path].getY()*10, 0));
                }else if(act_index > grid[x_path-1][y_path] && grid[x_path-1][y_path] > 1){
                    x_path = x_path - 1;
                    path.push_back(Point(map.get_coordinates()[x_path][y_path].getX()*10, map.get_coordinates()[x_path][y_path].getY()*10, 0));
                }
                act_index = grid[x_path][y_path];
                // std::cout << "act_index " << act_index << std::endl;
            }
            std::cout << "found goal" << std::endl;
            return path;
        }
        // Explore neighbors (up, down, left, right)
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if(abs(dy) == abs(dx)){
                    continue;
                }
                int newX = current.getX() + dx;
                int newY = current.getY() + dy;
                // X 0 X
                // 0 2 0
                // X 0 X
                // Check for valid neighbors within grid bounds and not walls
                if (newX >= 0 && newX < cols && newY >= 0 && newY < rows && grid[newX][newY] == 0) {

                    // X 0 X
                    // 0 2 0
                    // X 0 X
                    //pick 1 point from 4 points
                    Point neighbor = Point(newX, newY, 0); // Increment distance
                    frontier.push(neighbor);
                    // X 0 4 0 X
                    // 0 4 3 4 0
                    // 4 3 2 3 4
                    // 0 A 3 4 0
                    // X 0 4 0 X
                    // A represent actual picked neighbor and check lower index from 4 neighbors  << that was before

                    // next index is my index + 1
                    next_index = grid[current.getX()][current.getY()]+1;
                    map.update_map(Point(map.get_coordinates()[newX][newY].getX(), map.get_coordinates()[newX][newY].getY(), 0), next_index);
                    grid[newX][newY] = next_index;
                }
            }
        }
    }
    return {};
}

/**
 *  nahradit 379 -418 map.update_map(Point(map.get_coordinates()[newX][newY].getX(), map.get_coordinates()[newX][newY].getY(), 0), current.distance + 1);]
 * 354-369 nechgeckovat o 2 policka bo to je zbytocne
 * Priority queue zmenit na queue
*/
