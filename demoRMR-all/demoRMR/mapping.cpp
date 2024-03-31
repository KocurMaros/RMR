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
void Mapping::create_map(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta){
    for(size_t u = 0; u < map_vector.size(); u++){
        double angle, distance;
        double obstacle_x, obstacle_y;
        uint16_t dim = map_vector[u].get_map_dimension();
        dim = 300;
        for(size_t i = 0; i < laser_data.numberOfScans; i++)
        {
            if(laser_data.Data[i].scanDistance == 0 || laser_data.Data[i].scanDistance > 3200){
                continue;
            }
       
            angle = shift_theta(shift_theta_robot(robotTheta) + laser_data.Data[i].scanAngle);
            distance = laser_data.Data[i].scanDistance/10.0;
            obstacle_x = robotX +  distance * cos(deg2rad(angle));
            obstacle_y = robotY +  distance * sin(deg2rad(angle));
            if(obstacle_x < map_vector[u].get_minX() || obstacle_x > map_vector[u].get_maxX() || obstacle_y < map_vector[u].get_minY() || obstacle_y > map_vector[u].get_maxY())
                continue;
            Point point = Point(obstacle_x, obstacle_y, 0);
            map_vector[u].update_map(point, true);         
        }
    }
}

void Mapping::Gmapping(LaserMeasurement laser_data, double robotX, double robotY, double robotTheta)
{
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
                // std::cout << arg1 << " " << arg2 << std::endl;
                // Point point(arg1, arg2, 0);
                map.update_map(Point(arg1,arg2,0), 1);
                map.update_map(Point(arg1,arg2+10,0), 1);
                map.update_map(Point(arg1,arg2-10,0), 1);
                map.update_map(Point(arg1+10,arg2,0), 1);
                map.update_map(Point(arg1-10,arg2,0), 1);
                map.update_map(Point(arg1,arg2+20,0), 1);
                map.update_map(Point(arg1,arg2-20,0), 1);
                map.update_map(Point(arg1+20,arg2,0), 1);
                map.update_map(Point(arg1-20,arg2,0), 1);
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
    std::cout << centerX << " " << centerY << " " << width << std::endl;
    Hash_map whole_map = Hash_map(centerX, centerY, 0, 10, width);
    for(size_t i = 0; i < map_vector.size(); i++){
        std::vector<std::vector<uint16_t>> map = map_vector[i].get_hash_map();
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
    std::ofstream MyFile(map_file);

    MyFile << centerX << " " << centerY << " " << width << std::endl;
    std::vector<std::vector<uint16_t>> map1 = whole_map.get_hash_map();
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
            // if(map1[j][k] == 1){
            //     std::cout << "X ";
            // }
            // else{
            //     std::cout << "";
            // }
        }
        std::cout << "|  ";
        // for(size_t k = 0; k < coordinates[j].size(); k++){
        //     std::cout << "[" << coordinates[j][k].getX() << "," << coordinates[j][k].getY() << "]";
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
    for (size_t i = 0; i < map.get_map_dimension(); i++)
    {
        if (map.get_coordinates()[i][0].getX() <= x && map.get_coordinates()[i + 1][0].getX() > x)
        {
            std::cout << map.get_coordinates()[i][0].getX() << " " << map.get_coordinates()[i + 1][0].getX() << std::endl;
            ind_goal_x = i;
            break;
        }
    }
    for(size_t j = 0; j < map.get_map_dimension(); j++)
    {
        if (map.get_coordinates()[0][j].getY() <= y && map.get_coordinates()[0][j + 1].getY() > y)
        {
            std::cout << map.get_coordinates()[0][j].getY() << " " << map.get_coordinates()[0][j + 1].getY() << std::endl;
            ind_goal_y = j;
            break;
        }
    }
    x = start.getX();
    y = start.getY();
    for (size_t i = 0; i < map.get_map_dimension(); i++)
    {
        if (map.get_coordinates()[i][0].getX() <= x && map.get_coordinates()[i + 1][0].getX() > x)
        {
            ind_start_x = i;
            break;
        }
    }
    for(size_t j = 0; j < map.get_map_dimension(); j++)
    {
        if (map.get_coordinates()[0][j].getY() <= y && map.get_coordinates()[0][j + 1].getY() > y)
        {
            ind_start_y = j;
            break;
        }
    }
    // std::cout << "Point coord" << start.getX() << " " << start.getY() << " " << goal.getX() << " " << goal.getY() << std::endl;
    // std::cout << "ind_start_x " << ind_start_x << " ind_start_y " << ind_start_y << " ind_goal_x " << ind_goal_x << " ind_goal_y " << ind_goal_y << std::endl;
    // std::cout << "Map ind " << map.get_coordinates()[ind_start_x][ind_start_y].getX() << " " << map.get_coordinates()[ind_start_x][ind_start_y].getY() << " " << map.get_coordinates()[ind_goal_x][ind_goal_y].getX() << " " << map.get_coordinates()[ind_goal_x][ind_goal_y].getY() << std::endl; 
    
    std::vector<Point> path = floodFillPathfind(ind_start_x, ind_start_y, ind_goal_x, ind_goal_y);
    // std::cout << "path size " << path.size() << std::endl;
    std::vector<Point> filteredPath;
    bool sameX = false, sameY = false;
    for(size_t i = 0; i < path.size(); i++){
        if(path[i].getX() == path[i+1].getX() && !sameY)
            sameX = true;
        if(path[i].getY() == path[i+1].getY() && !sameX)
            sameY = true;
        
        if(path[i].getX() == path[i+1].getX() && path[i].getY() != path[i+1].getY() && sameY){
            filteredPath.push_back(path[i]);
            sameY = false;
            continue;
        }
        if(path[i].getY() == path[i+1].getY() && path[i].getX() != path[i+1].getX() && sameX){
            filteredPath.push_back(path[i]);
            sameX = false;
            continue;
        }

    }
    return filteredPath;
    // std::cout << "filteredPath size " << filteredPath.size() << std::endl;

    // for(size_t i = 0; i < filteredPath.size(); i++){
    //     std::cout << filteredPath[i].getX() << " " << filteredPath[i].getY() << std::endl;
    // }
}

std::vector<Point> Mapping::floodFillPathfind(int startX, int startY, int goalX, int goalY) {
    std::vector<std::vector<uint16_t>> grid = map.get_hash_map();

    int lowest_index = 3;
    
    int rows = map.get_map_dimension();
    int cols = map.get_map_dimension();

    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::priority_queue<PointQueue> frontier;

    PointQueue start = {goalX, goalY, 0};
    frontier.push(start);
    visited[goalX][goalY] = true;

    while (!frontier.empty()) {
        PointQueue current = frontier.top();
        frontier.pop();

        if (current.x == startX && current.y == startY) {
            std::cout << "found goal" << std::endl;
            uint16_t act_index;
            int x_path = current.x;
            int y_path = current.y;
            grid = map.get_hash_map();
            act_index = grid[x_path][y_path];
            std::vector<Point> path;
            path.push_back(Point(map.get_coordinates()[x_path][y_path].getX()*10, map.get_coordinates()[x_path][y_path].getY()*10, 0));
            while(act_index != 3){
                if(act_index > grid[x_path-1][y_path] && grid[x_path-1][y_path] >=2 && grid[x_path-2][y_path] >= 2){
                    x_path = x_path - 1;
                    path.push_back(Point(map.get_coordinates()[x_path][y_path].getX()*10, map.get_coordinates()[x_path][y_path].getY()*10, 0));
                }
                else if(act_index > grid[x_path+1][y_path] && grid[x_path+1][y_path] >=2&& grid[x_path+2][y_path] >= 2){
                    x_path = x_path + 1;
                    path.push_back(Point(map.get_coordinates()[x_path][y_path].getX()*10, map.get_coordinates()[x_path][y_path].getY()*10, 0));
                }
                else if(act_index > grid[x_path][y_path-1] && grid[x_path][y_path-1] >=2&& grid[x_path][y_path-2] >= 2){  
                    y_path = y_path - 1;
                    path.push_back(Point(map.get_coordinates()[x_path][y_path].getX()*10, map.get_coordinates()[x_path][y_path].getY()*10, 0));
                }
                else if(act_index > grid[x_path][y_path+1] && grid[x_path][y_path+1] >=2&& grid[x_path][y_path+2] >= 2){
                    y_path = y_path + 1;
                    path.push_back(Point(map.get_coordinates()[x_path][y_path].getX()*10, map.get_coordinates()[x_path][y_path].getY()*10, 0));
                }
                // std::cout << grid[x_path][y_path] << " " << grid[x_path-1][y_path] << " " << grid[x_path+1][y_path] << " " << grid[x_path][y_path-1] << " " << grid[x_path][y_path+1] << std::endl;
                // std::cout << "x_path " << x_path << " y_path " << y_path << std::endl;
                act_index = grid[x_path][y_path];
            }
            return path;
            // break;
            // std::cout << current.x << " " << current.y << std::endl;
            // std::vector<PointQueue> path;
            // path.push_back(current);
   
            // // Backtrack to reconstruct path
            // PointQueue currentForPath = current;
            // while (visited[currentForPath.y][currentForPath.x]) { // Loop until we reach the starting point
            //     // Optional: If using parent field in PointQueue
            //     // currentForPath = currentForPath.parent;
            //     // Search visited grid for predecessor (point with distance-1)
            //     for (int dx = -1; dx <= 1; dx++) {
            //         for (int dy = -1; dy <= 1; dy++) {
            //             int prevX = currentForPath.x + dx;
            //             int prevY = currentForPath.y + dy;
            //             std::cout << "prevX " << prevX << " prevY " << prevY << std::endl;
            //             if (prevX >= 0 && prevX < cols && prevY >= 0 && prevY < rows && 
            //                 visited[prevY][prevX] && grid[prevY][prevX] != 1 && 
            //                 visited[prevY][prevX] == currentForPath.distance - 1) {
            //                 path.push_back({prevX, prevY, currentForPath.distance - 1});
            //                 currentForPath = {prevX, prevY, currentForPath.distance - 1};
            //                 break; // Exit inner loop after finding predecessor
            //             }
            //         }
            //     }
            // }
            // std::cout << "path size " << path.size() << std::endl;
            // // Reverse the path for correct order (goal to start)
            // std::reverse(path.begin(), path.end());
            // return path;
        }

        // Explore neighbors (up, down, left, right)
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if(abs(dy) == abs(dx)){
                    continue;
                }
                int newX = current.x + dx;
                int newY = current.y + dy;

                // Check for valid neighbors within grid bounds and not walls
                if (newX >= 0 && newX < cols && newY >= 0 && newY < rows && grid[newX][newY] != 1 && !visited[newX][newY]) {
                    visited[newX][newY] = true;
                    PointQueue neighbor = {newX, newY, current.distance + 1}; // Increment distance
                    frontier.push(neighbor);
                    
                    grid = map.get_hash_map();
                    
                    lowest_index = 3;
                    for (int dxx = -1; dxx <= 1; dxx++) {
                        for (int dyy = -1; dyy <= 1; dyy++) {
                            if(abs(dyy) == abs(dxx)){
                                continue;
                            }
                            int prevX = newX + dxx;
                            int prevY = newY + dyy;
                            if (prevX >= 0 && prevX < cols && prevY >= 0 && prevY < rows && grid[prevX][prevY] != 1 && 
                                grid[prevX][prevY] >= lowest_index) {
                                // std::cout << lowest_index << " " << grid[prevX][prevY] << std::endl;
                                lowest_index = grid[prevX][prevY]+1;
                            }
                        }
                    }
                    map.update_map(Point(map.get_coordinates()[newX][newY].getX(), map.get_coordinates()[newX][newY].getY(), 0), lowest_index);
                }
            }
        }
    }

    // No path found
    return {};
}