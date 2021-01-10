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
        //Servo s2; // Used later once I figure out what servo goes where
        int H0;
        int H1;
        // int H2;
        int H3;
        int H4;
        int H5;

};

#endif //ADEEPT_HEXAPOD_LEG_H
