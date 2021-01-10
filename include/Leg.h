//
// Created by timmy on 1/10/21.
//

#ifndef ADEEPT_HEXAPOD_LEG_H
#define ADEEPT_HEXAPOD_LEG_H

#include <Servo.h>
#include <SoftwareSerial.h>
#include<Wire.h>
#include <SR04.h>
#include <Good_code.h>
#include <Shitty_Code.h>

class leg{
    public:
        Servo s1;
        //Servo l2.s1; // Used later once I figure out what servo goes where
        int b_counter = 0; // two counters for now until I know its safe for them to be 1
        int f_counter = 0;
        void Step_fw(int val);
        void Step_bw(int val);

};

#endif //ADEEPT_HEXAPOD_LEG_H
