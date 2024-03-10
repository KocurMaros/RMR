#include "controller.h"
#define PI          3.14159 /* pi */

// Implement your controller functions here
PIController::PIController(double kp, double ki, double kp_rot) : kp_(kp), ki_(ki),kp_rot_(kp_rot), integral_(0.0) {
    ramp = Ramp();
}

PIController::~PIController() {}


void PIController::compute(Point actual_point, Point desired_point, double dt_, int *trans_speed, double *rot_speed) {
    
    double actual_x, actual_y, actual_theta;
    double desired_x, desired_y, desired_theta;
    actual_x = actual_point.getX();
    actual_y = actual_point.getY();
    actual_theta = actual_point.getTheta();
    desired_x = desired_point.getX();
    desired_y = desired_point.getY();
    desired_theta = desired_point.getTheta();


    error_distance = sqrt(pow(desired_x - actual_x, 2) + pow(desired_y - actual_y, 2));
    error_angle = atan2(desired_y - actual_y, desired_x - actual_x);
    std::cout << "atan2 = " << error_angle << std::endl;
    error_angle = error_angle - actual_theta;
    std::cout << "actual = " << actual_theta << std::endl;
    // double error_angle = atan2(desired_.y - actual_.y, desired_.x - actual_.x);

    if (error_angle > PI) {
        error_angle -= 2 * PI;
    } else if (error_angle <= -PI) {
        error_angle += 2 * PI;
    }
    std::cout << "error angle = " << error_angle << std::endl;

    integral_ = integral_ + error_distance*dt_;

    double omega = kp_*error_distance + ki_*integral_;
    double omega_rot = kp_rot_*error_angle;


    if(abs(omega) > MAX_SPEED){
        clearIntegral();
        if(omega>0){
            omega = MAX_SPEED;
        }
        else {
            omega = -MAX_SPEED;
        }
    }
    if(abs(omega_rot) > MAX_SPEED_ROT){
        if (omega_rot > 0){
            omega_rot = MAX_SPEED_ROT;
        }
        else {
            omega_rot = -MAX_SPEED_ROT;
        }
    }


    ramp.compute(&omega, &omega_rot, 0.01);
    // std::cout << omega << " " << omega_rot << " " << std::endl;
    *rot_speed = omega_rot;
    *trans_speed =static_cast<int> (omega);
    if(abs(omega) <= 10 && abs(omega_rot) <= PI/180*2){ //ak je spped mensai nez 10mm/sec i rotacna < 1 Rad
        ramp.clear_time();
        std::cout << "clear time" << std::endl;
    }
}
