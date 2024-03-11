#include "ramp.h"

Ramp::Ramp() {}

Ramp::~Ramp() {}

//create map function from 0 - 1 sec to speed - max_sped
void Ramp::compute(double *speed, double *speed_rot, double change_speed) {
    if(current_speed_multiplier > max_speed){
        return;
    }
    current_speed_multiplier += change_speed;
    *speed *= current_speed_multiplier;
    *speed_rot *= current_speed_multiplier/2.0;
}
void Ramp::clear_time(){
    if(current_speed_multiplier > 20.0)
        current_speed_multiplier = 0;
}

void Ramp::clear_time_hard(){
    current_speed_multiplier = 0;
}
