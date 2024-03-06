#include "ramp.h"

Ramp::Ramp() {
    current_time = 0.0;
}

Ramp::~Ramp() {}

Point Ramp::compute(Point actual_point, double dt) {
    Point desired_point;
 
    if (current_time < ramp_time) {
        speed = distance / ramp_time;
        radius = angle / ramp_time;
    } else {
        speed = distance;
        radius = angle;
    }
    current_time += dt;
    return Point(speed, radius);
}


Ramp::set_start_conditions(Point start_point, Point end_point) {
    start_point = start_point;
    end_point = end_point;
    distance = sqrt(pow(end_point.getX() - start_point.getX(), 2) + pow(end_point.getY() - start_point.getY(), 2));
    angle = atan2(end_point.getY() - start_point.getY(), end_point.getX() - start_point.getX()) - start_point.getTheta();
}