#ifdef RAMP_H
#define RAMP_H

#include "point.h"

class Ramp {
public:
    Ramp();
    ~Ramp();

    Point compute(Point actual_point, double dt);
    void set_start_conditions(Point start_point, Point end_point);

private:
    double current_time;
    double ramp_time = 1.0;  //to max speed in 1 second
    Point start_point;
    Point end_point;
    double distance;
    double angle;
}

#endif