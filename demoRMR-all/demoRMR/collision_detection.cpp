#include "collision_detection.h"
#include<iostream>

#define MAX_ZONE_DISTANCE 1.0

bool CollisionDetection::isObstacleInPathStatic(double scanDistance, double scanAngle, double zoneAngle, double zoneDistance){
    //distances su v metroch
    //vsetky vstupy su v STUPNOCH
    //zoneAngle je relativny od osi robota lavotocivy, -180 az 180, scanAngle - nula je vpredu robota, pravotocivy ide od 0 od 360

    //normalizacia uhlu z lidaru
    scanAngle = normalizeLidarAngle(scanAngle);


    //normalizacia angle erroru v zone
    double errorAngle = scanAngle - zoneAngle;
    if (errorAngle >= 180) errorAngle -= 360;
    else if (errorAngle < -180) errorAngle += 360;

    //vypocet ci je prekazka v ceste, ak je scanDistance 0 - v max range lidaru nie je objekt a teda neni tam prekazka
    //taktiez tam neni prekazka, ked je v lidare nieco dalej ako je kriticka vzdialenost
    if (scanDistance!=0.0 && errorAngle<=90 && errorAngle >=-90) {
        //TODO: nahrad za PI
        double maxDistance = sqrt(pow(zoneDistance+CRITICAL_DISTANCE,2)+pow(CRITICAL_DISTANCE,2));
        double scanCritical;
        if (errorAngle != 0.0){
            scanCritical = CRITICAL_DISTANCE / sin(fabs(errorAngle) * PI / 180);
        }
        else {
            scanCritical = zoneDistance+CRITICAL_DISTANCE;
        }

        if (scanCritical > maxDistance){
            scanCritical = maxDistance;
        }
        if (scanDistance <= scanCritical){
            return true;
        }
    }
    return false;
}

bool CollisionDetection::isObstacleInPath(double scanDistance, double scanAngle, double zoneAngle, double zoneDistance) {
    //distances su v metroch
    //vsetky vstupy su v STUPNOCH
    //zoneAngle je relativny od osi robota lavotocivy, -180 az 180, scanAngle - nula je vpredu robota, pravotocivy ide od 0 od 360

    //normalizacia uhlu z lidaru
    scanAngle = normalizeLidarAngle(scanAngle);
    if (zoneDistance>MAX_ZONE_DISTANCE)
    zoneDistance = MAX_ZONE_DISTANCE;


    //normalizacia angle erroru v zone
    double errorAngle = scanAngle - zoneAngle;
    if (errorAngle >= 180) errorAngle -= 360;
    else if (errorAngle < -180) errorAngle += 360;


    //vypocet ci je prekazka v ceste, ak je scanDistance 0 - v max range lidaru nie je objekt a teda neni tam prekazka
    //taktiez tam neni prekazka, ked je v lidare nieco dalej ako je kriticka vzdialenost
    if (scanDistance!=0.0 && errorAngle<=90 && errorAngle >=-90) {
        //TODO: nahrad za PI
        double maxDistance = sqrt(pow(zoneDistance+CRITICAL_DISTANCE,2)+pow(CRITICAL_DISTANCE,2));
        double scanCritical;
        if (errorAngle != 0.0){
            scanCritical = CRITICAL_DISTANCE / sin(fabs(errorAngle) * PI / 180);
        }
        else {
            scanCritical = zoneDistance+CRITICAL_DISTANCE;
        }

        if (scanCritical > maxDistance){
            scanCritical = maxDistance;
        }
        if (scanDistance <= scanCritical){
            obstacle.setDistance(scanDistance);
            obstacle.setAngle(scanAngle);
            return true;
        }
    }
    return false;
}

double CollisionDetection::normalizeLidarAngle(double angle){
    if (angle > 180)
        angle = 360 - angle;
    else angle = - angle;

    return angle;
}

void Obstacle::calculateLeftEdgePoint(double robotX, double robotY, double robotFi) {
    double c = sqrt(pow(getLeftEdge()->getDistance(),2) - pow(CRITICAL_DISTANCE*3/2,2));
    double alfa = asin(CRITICAL_DISTANCE*3/2/getLeftEdge()->getDistance())+ getLeftEdge()->getAngle() * PI / 180;

    // std::cout << "asin: " << asin(CRITICAL_DISTANCE*3/2/getLeftEdge()->getDistance())*180/PI << std::endl;
    // std::cout << "alfa: " << alfa*180/PI << std::endl;
    // std::cout << "left edge angle: " << getLeftEdge()->getAngle() << std::endl;

    alfa = alfa + robotFi*PI/180;

    if (alfa >= PI) alfa -= 2*PI;
    else if (alfa < -PI) alfa += 2*PI;


    // mozno poriesit tieto suradnicove systemy a tak dalej ..... netusim where chyba uz naozaj :D

    c = c + CRITICAL_DISTANCE;
    double x = robotX +  c * cos(alfa);
    double y = robotY + c * sin(alfa);

    // std::cout << "c: " << c << std::endl;
    // std::cout << "x: " << x << std::endl;
    // std::cout << "y: " << y << std::endl;
    // std::cout << "robotX: " << robotX << std::endl;
    // std::cout << "robotY: " << robotY << std::endl;
    getLeftEdge()->getPoint()->setPoint(x*1000,y*1000,0);
}

void Obstacle::calculateRightEdgePoint(double robotX, double robotY, double robotFi) {
    double c = sqrt(pow(getRightEdge()->getDistance(),2) - pow(CRITICAL_DISTANCE*3/2,2));
    double alfa = getRightEdge()->getAngle() * PI / 180 - asin(CRITICAL_DISTANCE*3/2/getRightEdge()->getDistance());

    // std::cout << "asin: " << asin(CRITICAL_DISTANCE*3/2/getLeftEdge()->getDistance())*180/PI << std::endl;
    // std::cout << "alfa: " << alfa*180/PI << std::endl;
    // std::cout << "right edge angle: " << getLeftEdge()->getAngle() << std::endl;

    alfa = alfa + robotFi*PI/180;

    if (alfa >= PI) alfa -= 2*PI;
    else if (alfa < -PI) alfa += 2*PI;

    c = c + CRITICAL_DISTANCE;
    double x = robotX +  c * cos(alfa);
    double y = robotY + c * sin(alfa);

    // std::cout << "c: " << c << std::endl;
    // std::cout << "x: " << x << std::endl;
    // std::cout << "y: " << y << std::endl;
    // std::cout << "robotX: " << robotX << std::endl;
    // std::cout << "robotY: " << robotY << std::endl;
    getRightEdge()->getPoint()->setPoint(x*1000,y*1000,0);
}

void CollisionDetection::resetCollisionDetection(){
    obstacle.setFoundObstacle(false);
    obstacle.getLeftEdge()->setFoundEdge(false);
    obstacle.getRightEdge()->setFoundEdge(false);
    obstacle.getLeftEdge()->setPointFree(false);
    obstacle.getRightEdge()->setPointFree(false);
}
