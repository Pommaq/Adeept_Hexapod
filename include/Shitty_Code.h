//
// Created by timmy on 1/9/21.
//

#ifndef ADEEPT_HEXAPOD_SHITTY_CODE_H
#define ADEEPT_HEXAPOD_SHITTY_CODE_H

// Prototypes to make the platformio compiler stfu
void battery();

void sendultrasonic();

long ultrasonic();

void sendbattery();

void original_latest();

void move_backward_latest1();

void move_forward_latest1();

void move_left_latest1();

void move_right_latest1();

void set_servo_back(Servo& servo, int &position, double value);

void set_servo_forw(Servo& servo, int &position, double value);

void Stand_Up();

long ultrasonic();

void step_by_step();

void doWhatever();

void attack();

char judgement(String &comdata);

void advoid();

#endif //ADEEPT_HEXAPOD_SHITTY_CODE_H
