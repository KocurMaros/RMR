#include "controller.h"

// Implement your controller functions here
PIController::PIController(double kp, double ki) : kp_(kp), ki_(ki), integral_(0.0) {
    kp_ = kp;
    ki_ = ki;
    integral_ = 0.0;
}

PIController::~PIController() {}


// double PIController::compute(Point *point, double dt_, double &speed, double &radius) {
 double compute(double desired_x,double desired_y, double desired_theta,
                   double actual_x,double actual_y, double actual_theta,
                   double dt_, double &speed, double &radius){   
    // double actual_x, actual_y, actual_theta;
    // double desired_x, desired_y, desired_theta;
    // point.getPointActual(actual_x, actual_y, actual_theta);
    // point.getPointDesire(desired_x, desired_y, desired_theta);

    double error_distance = sqrt(pow(desired_x - actual_x, 2) + pow(desired_y - actual_y, 2));
    double error_angle = atan2(desired_y - actual_y, desired_x - actual_x) - actual_theta;
    // double error_angle = atan2(desired_.y - actual_.y, desired_.x - actual_.x);

    integral_ = integral_ + error_distance*dt_;

    double omega = kp_*error_distance + ki_*integral_;
    cout << error_angle << " " << error_distance << " " << omega << endl;
    radius = (int) (error_distance / (2*sin(error_angle)));
    speed =(int) (omega*param.radius);
}
