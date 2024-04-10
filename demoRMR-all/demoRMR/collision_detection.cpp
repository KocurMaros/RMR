#include "collision_detection.h"
#define PI          3.14159 /* pi */
#include<iostream>

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

void Obstacle::calculateLeftEdgePoint(double robotX, double robotY) {
    double c = sqrt(pow(getLeftEdge()->getDistance(),2) + pow(CRITICAL_DISTANCE*3/2,2));
    double alfa = asin(CRITICAL_DISTANCE*3/2/c)+ getLeftEdge()->getAngle() * PI / 180;

    if (alfa >= 180) alfa -= 360;
    else if (alfa < -180) alfa += 360;

    //check if it is normal
    // c = c + CRITICAL_DISTANCE+0.01;
    double x = robotX +  c * cos(alfa);
    double y = robotY + c * sin(alfa);
    getLeftEdge()->getPoint()->setPoint(x*1000,y*1000,0);
}

void Obstacle::calculateRightEdgePoint(double robotX, double robotY) {
    double c = sqrt(pow(getRightEdge()->getDistance(),2) + pow(CRITICAL_DISTANCE+0.01,2));
    double alfa = getRightEdge()->getAngle() * PI / 180 - asin(CRITICAL_DISTANCE+0.01/c);

    if (alfa >= 180) alfa -= 360;
    else if (alfa < -180) alfa += 360;

    c = c + CRITICAL_DISTANCE+0.01;
    double x = robotX +  c * cos(alfa);
    double y = robotY + c * sin(alfa);
    getRightEdge()->getPoint()->setPoint(x*1000,y*1000,0);
}

