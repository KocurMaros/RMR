#ifndef CONTROLLER_H
#define CONTROLLER_H

// #include "point.h"
#include <iostream>
#include <math.h>
class PIController {
public:
    PIController(double kp, double ki);
    ~PIController();

    // double compute(Point point, double dt_, double &speed, double &radius);
    double compute(double desired_x,double desired_y, double desired_theta,
                   double actual_x,double actual_y, double actual_theta,
                   double dt_, double &speed, double &radius);
    void clearIntegral() { integral_ = 0.0; }
private:
    
    double kp_ = 100.0;
    double ki_ = 1.0;
    double integral_ = 0.0;
};

#endif // CONTROLLER_H
