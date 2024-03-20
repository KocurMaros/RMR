#include <math.h>
#include "collision_detection.h"

#define CRITICAL_DISTANCE 0.2 //[m]


bool isObstacleInPath(double scanDistance, double scanAngle, double zoneAngle, double zoneDistance) {
    //distances su v metroch
    //vsetky vstupy su v STUPNOCH
    //zoneAngle je relativny od osi robota lavotocivy, -180 az 180, scanAngle - nula je vpredu robota, pravotocivy ide od 0 od 360

    //normalizacia uhlu z lidaru
    if (scanAngle > 180)
        scanAngle = 360 - scanAngle;
    else scanAngle = - scanAngle;


    //normalizacia angle erroru v zone
    double errorAngle = scanAngle - zoneAngle;
    if (errorAngle >= 180) errorAngle -= 360;
    else if (errorAngle < -180) errorAngle += 360;


    if (scanDistance!=0.0 && errorAngle<=90 && errorAngle >=-90) {
        //TODO: nahrad za PI
        double maxDistance = sqrt(pow(zoneDistance,2)+pow(CRITICAL_DISTANCE,2));
        double scanCritical = maxDistance;
        if (errorAngle != 0.0){
            scanCritical = CRITICAL_DISTANCE / sin(fabs(errorAngle) * 3.14 / 180);
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
