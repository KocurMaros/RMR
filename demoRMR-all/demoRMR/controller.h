#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "point.h"
#include <iostream>
#include <math.h>
#include "ramp.h"

#define MAX_SPEED       400//mm/s
#define MAX_SPEED_ROT   3.14//rad/s
class PIController {
public:
     PIController(double kp, double ki, double kp_rot);
    ~PIController();
     double error_distance;
     double error_angle;

    void compute(Point actual_point, Point desired_point, double dt_, int *trans_speed, double *rot_speed);
    // double compute(double desired_x,double desired_y, double desired_theta,
    //                double actual_x,double actual_y, double actual_theta,
    //                double dt_, double &speed, double &radius);
    void clearIntegral() { integral_ = 0.0; }
    void clearErrors() {error_angle = 0.0; error_distance = 0.0;}
private:
    Ramp ramp;
    double kp_;
    double kp_rot_;
    double ki_;
    double integral_;
};

#endif // CONTROLLER_H
