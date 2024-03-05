#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "point.h"
#include <iostream>
#include <math.h>

#define MAX_SPEED       400//mm/s
class PIController {
public:
     PIController(double kp, double ki, double kp_rot);
    ~PIController();

    void compute(Point point, double dt_, int *trans_speed, int *rot_speed);
    // double compute(double desired_x,double desired_y, double desired_theta,
    //                double actual_x,double actual_y, double actual_theta,
    //                double dt_, double &speed, double &radius);
    void clearIntegral() { integral_ = 0.0; }
private:
    
    double kp_;
    double kp_rot_;
    double ki_;
    double integral_;
};

#endif // CONTROLLER_H
