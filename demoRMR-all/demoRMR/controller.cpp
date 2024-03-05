#include "controller.h"

// Implement your controller functions here
PIController::PIController(double kp, double ki, double kp_rot) : kp_(kp), ki_(ki),kp_rot_(kp_rot), integral_(0.0) {}

PIController::~PIController() {}


void PIController::compute(Point point, double dt_, int *trans_speed, int *rot_speed) {
//  double compute(double desired_x,double desired_y, double desired_theta,
//                    double actual_x,double actual_y, double actual_theta,
//                    double dt_, double &speed, double &radius){   
    double actual_x, actual_y, actual_theta;
    double desired_x, desired_y, desired_theta;
    point.getPointActual(&actual_x,  &actual_y, &actual_theta);
    point.getPointDesire(&desired_x, &desired_y, &desired_theta);

    double error_distance = sqrt(pow(desired_x - actual_x, 2) + pow(desired_y - actual_y, 2));
    double error_angle = atan2(desired_y - actual_y, desired_x - actual_x) - actual_theta;
    // double error_angle = atan2(desired_.y - actual_.y, desired_.x - actual_.x);

    integral_ = integral_ + error_distance*dt_;

    double omega = kp_*error_distance + ki_*integral_;
    double omega_rot = kp_rot_*error_angle;
    if(omega > MAX_SPEED){
        clearIntegral();
        omega = MAX_SPEED;
    }
    if(omega_rot > MAX_SPEED/2)
        omega_rot = MAX_SPEED/2;
    //std::cout << error_angle << " " << error_distance << " " << omega << std::endl;
    *rot_speed = static_cast<int> (omega_rot);
    *trans_speed =static_cast<int> (omega);
}
