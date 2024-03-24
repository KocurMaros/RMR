#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H
#include <math.h>

class CollisionDetection {
public:
    bool isObstacleInPath(double scanDistance, double scanAngle, double zoneAngle, double zoneDistance);
};

#endif // COLLISION_DETECTION_H
