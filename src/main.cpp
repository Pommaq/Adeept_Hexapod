
/***********************************************************
  File name:  Adeept_Hexapod.ino
  Description:
  1. wifi control mode: control with wifi controller
  2. Press different keys to realize the left and
  right movement, automatic obstacle avoidance and
  self-stabilization of Quadruped Robot

  Website: www.adeept.com
  E-mail: support@adeept.com
  Author: Felix
  Date: 2019/07/01
***********************************************************/

//add some libraries

#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include<Wire.h>
#include <SR04.h>
#include <Good_code.h>
#include <Shitty_Code.h>
#include <Leg.h>
// Define leg class. TODO: Move this into separate file



//define pins
#define PIN  A1
#define led_numbers  6
#define TRIG_PIN A2
#define ECHO_PIN A3
Adafruit_NeoPixel strip = Adafruit_NeoPixel(led_numbers, PIN, NEO_GRB + NEO_KHZ800);
SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);


//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
int lcd_loop_counter;
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output, angle_yaw_output;
float angle_changed;
// long a; // Used in one function call.

// Create servo objects
leg l0;
leg l1;
leg l2;
leg l3;
leg l4;
leg l5;
leg l6;
leg l7;
leg l8;
leg l9;
leg l10;
leg l11;
leg l12;
/*
Servo l0.s1;
Servo l1.s1;
Servo l2.s1;
Servo l3.s1;
Servo l4.s1;
Servo l5.s1;
Servo l6.s1;
Servo l7.s1;
Servo l8.s1;
Servo l9.s1;
Servo l10.s1;
Servo l11.s1;
Servo l12.s1;
*/

// These oddly named integers appears to store the current "location" of each servo.
int i0H0 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i1H0 = 0; // used 46*2 times in one of the loops it seems. // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i2H0 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i3H0 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i4H0 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i5H0 = 0;// Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i6H0 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i7H0 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i8H0 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i9H0 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i10H0 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i11H0 = 0;  // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...

int i0H1 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i1H1 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i2H1 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i3H1 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i4H1 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i5H1 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i6H1 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i7H1 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i8H1 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i9H1 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i10H1 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i11H1 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i12H1 = 0; // Global variable used for a local purpose. TODO: Remove this.


int i0H3 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i1H3 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i2H3 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i3H3 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i4H3 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i5H3 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i6H3 = 0;  // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i7H3 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i8H3 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i9H3 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i10H3 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i11H3 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...


int i2H4 = 0; // used in attack() but gets zeroed afterwards. // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i6H4 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i10H4 = 0;  // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i12H4 = 0; // Another global variable used for a local purpose. TODO: Remove it.

int i0H5 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i4H5 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i8H5 = 0; // Its going to be set to 0 (or 1 due to what i think is a bug) after usage...
int i12H5 = 0; // Global variable for a local purpose. TODO: Remove it

int battery_voltage;
String comdata = "";

String text1 = "1";
String text2 = "2";
String text3 = "3";
String text4 = "4";
String text5 = "5";
String text6 = "10";
String text7 = "7";
String text8 = "8";
String text9 = "9";

String phone1 = "forwardStart\n";
String phone2 = "backwardStart\n";
String phone3 = "leftStart\n";
String phone4 = "rightStart\n";
String phone5 = "aStart\n";
String phone6 = "bStart\n";

String phone7 = "forwardStop\n";
String phone8 = "backwardStop\n";
String phone9 = "leftStop\n";
String phone10 = "rightStop\n";

#define ACTIONSPEED  1 // The delay in milliseconds between every time we make a servo do something
unsigned long uptime = 0;
int angle = 20;


void setup() {
    //initialize the steering gear
    l0.s1.attach(2);
    l1.s1.attach(3);
    l2.s1.attach(4);
    l3.s1.attach(5);
    l4.s1.attach(6);
    l5.s1.attach(7);
    l6.s1.attach(8);
    l7.s1.attach(9);
    l8.s1.attach(10);
    l9.s1.attach(11);
    l10.s1.attach(12);
    l11.s1.attach(13);
    l12.s1.attach(14);
    original_latest(); // Makes this sexy little spider stand up

    //calculate battery capacity
    battery_voltage = battery_voltage * 0.92 + ((analogRead(A7) * 2.93) + 250) * 0.08;

    //initialize the strip
    strip.begin();
    strip.setBrightness(5);
    for (int i = 0; i < 6; i++) {
        strip.setPixelColor(i, strip.Color(255, 210, 67));
        strip.show();
        delay(50);
    }
    ESP8266_ATCOMMAND();     //initialize the ESP8266
    self_balanced_setup();   //initialize the MPU6050

    for (int i = 0; i < 6; i++) {
        strip.setPixelColor(i, strip.Color(55, 122, 132));
        strip.show();
        delay(50);
    }

    //send the battery capacity data to ESP8266
    battery_voltage = battery_voltage * 0.92 + ((analogRead(A7) * 2.93) + 250) * 0.08;
    double showbattery = (battery_voltage - 190) / 2.93 * 5 / 1023 * 6;
    Serial.println("AT+CIPSEND=0,6\r\n");
    delay(10);
    Serial.print("V:");
    Serial.println(showbattery);
    delay(250);


}


void loop() {

    while (Serial.available() > 0) // TODO: See if we can simplify this.
    {
        comdata += char(Serial.read());
        delay(1);
    }

    char judge = judgement(comdata);
    Serial.println((int)judge);
    switch (judge) {
        case 1: while (!Serial.available()) {
                move_forward_latest1();
            }
            break;
        case 2: while (!Serial.available()) {
                move_backward_latest1();
            }
            break;
        case 3: while (!Serial.available()) {
                move_right_latest1();
            }
            break;
        case 4: while (!Serial.available()) {
                move_left_latest1();
            }
            break;
        case 5: while (!Serial.available()) { // Detta är varför roboten får spasmer ibland...
                self_balanced();
                self_balanced_test();
            }
            break;
        case 6: while (!Serial.available()) {
                attack();
            }
            break;
        case 8: while (!Serial.available()) { // User pressed "Auto". This is broken and appears to enter an infinite loop
                advoid();
            }
            break;
        case 9: while (!Serial.available()) {
                original_latest();
                sendbattery();
                battery();
            }
            break;
        case 11: while (!Serial.available()) { // Wtf gör den här?
                l0.s1.attach(2);
                l1.s1.attach(3);
                l2.s1.attach(4);
                l3.s1.attach(5);
                l4.s1.attach(6);
                l5.s1.attach(7);
                l6.s1.attach(8);
                l7.s1.attach(9);
                l8.s1.attach(10);
                l9.s1.attach(11);
                l10.s1.attach(12);
                l11.s1.attach(13);
                l12.s1.attach(14);
            }
            break;
        default:  break;
    }

    if (judge == 7) {                      //send the ultrasonic data to ESP8266
        Serial.println("AT+CIPSEND=0,4\r\n");
        delay(20);
        Serial.println("U:35");
        delay(250);
        for (int u = 0; u < 104; u++) {
            sendultrasonic();
        }
        Serial.println("AT+CIPSEND=0,1\r\n");
        delay(20);
        Serial.println("F");
        delay(250);
    }

    comdata = "";
}


void battery() { // Checks the current battery level and sets lights to indicate if its low or not
    // TODO: If we assume it will be charged for the most part, swap the if/else cases. Failed if-cases use two instructions compared to 1 upon success.
    if (battery_voltage < 860 ) {                    //If batteryvoltage is below 10.5V and higher than 8.0V 860
        for (int i = 0; i < 6; i++) {
            strip.setPixelColor(i, strip.Color(255, 0, 0)); // Red
            strip.show();
            delay(1);                                                //Turn on the led if battery voltage is to low

        }                                                //Turn on the led if battery voltage is to low
    }
    else {
        for (int i = 0; i < 6; i++) { // Battery level is OK.
            strip.setPixelColor(i, strip.Color(55, 122, 132)); // Blue-ish
            strip.show();
            delay(1);
        }
    }
}

void sendultrasonic() { // Sends an ultrasonic signal and rotates head to scan the surroundings.
    l12.s1.write(angle);
    angle++;
    if (angle % 3 == 0) {
        long a = ultrasonic();
        Serial.print("AT+CIPSEND=0,3\r\n");
        delay(20);
        Serial.println(a);
        delay(250);
    }
    if (angle == 124) {
        angle = 20;
    }
}


void sendbattery() {
    while (Serial.available() <= 0) {
        battery_voltage = battery_voltage * 0.92 + ((analogRead(A7) * 2.93) + 250) * 0.08; // Sets global variable battery_voltage to... the battery voltage
        double showbattery = (battery_voltage - 190) / 2.93 * 5 / 1023 * 6;
        if (millis() - uptime > 15000) { // If it's been more than 150 seconds since last time this function was called
            Serial.println("AT+CIPSEND=0,6\r\n");
            Serial.print("V:");
            Serial.println(showbattery);
            delay(10); // This delay is to guarantee there has been 10ms since last time we called millis(). This is required due to arduino hardware. A certain capacitor requires time to recharge.
            uptime = millis(); // sets uptime to the amount of time, in milliseconds, the arduino has been running
        }
    }
}
//This is the initialization angle of each leg, you can modify the following values according to your needs.
void original_latest() {
    l0.s1.write(90);//D2
    l1.s1.write(90);//D3
    l2.s1.write(90);//D4
    l3.s1.write(90);//D5
    l4.s1.write(90);//D6
    l5.s1.write(90); //D7
    l6.s1.write(90);//D8
    l7.s1.write(90); //D9
    l8.s1.write(90);//D10
    l9.s1.write(90); //D11
    l11.s1.write(90);//D12
    l10.s1.write(90);//D13
    l12.s1.write(90);//D14
}


void move_backward_latest1() {
    //PROCESS 5:
    // TODO: This is stupid. Fix it.
    int counters[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // We might not use all of them, but im having as many counters as servos so the indexes line up
    for (int i = 0; i <= 45; i++) {
        set_servo_back(l0.s1, counters[0], 90);
        l2.s1.write(90);
        delay(ACTIONSPEED);

        set_servo_back(l4.s1, counters[4], 90);
        l6.s1.write(90);
        delay(ACTIONSPEED);

        set_servo_forw(l8.s1, counters[8], 90);
        l10.s1.write(90);
        delay(ACTIONSPEED);
    }

    //PROCESS 3:

    for (int i = 0; i <= 45; i++) {
        // Move back to touch the ground1
        set_servo_forw(l0.s1, counters[0], 45);
        set_servo_back(l1.s1, counters[1], 90);
        delay(ACTIONSPEED);

        // Raise leg2
        l2.s1.write(90);
        set_servo_forw(l3.s1, counters[3], 45);
        set_servo_forw(l3.s1, counters[3], 45);
        delay(ACTIONSPEED);

        // Move back to touch the ground3
        set_servo_forw(l4.s1, counters[4], 45);
        set_servo_back(l5.s1, counters[5], 120);
        delay(ACTIONSPEED);

        // Rise the leg4
        l6.s1.write(90);
        set_servo_back(l7.s1, counters[7], 135);
        delay(ACTIONSPEED);

        // Move back to touch the ground5
        set_servo_back(l8.s1, counters[8], 135);
        set_servo_forw(l9.s1, counters[9], 72.5);
        delay(ACTIONSPEED);

        // Rise the leg6
        l10.s1.write(90);
        set_servo_back(l11.s1, counters[11], 90);
        delay(ACTIONSPEED);
    }

    /*********rotate backward to the initial position and rotate forward**********/

    //PROCESS 4:
    for (int i = 0; i <= 45; i++) {
        // TODO: Set this inside another loop.
        l0.s1.write(90);
        set_servo_back(l2.s1, counters[2], 90);
        delay(ACTIONSPEED);

        l4.s1.write(90);
        set_servo_forw(l6.s1, counters[6], 90);
        delay(ACTIONSPEED);

        l8.s1.write(90);
        set_servo_forw(l10.s1, counters[10], 90);
        delay(ACTIONSPEED);
    }

    //PROCESS 1:
    for (int i = 0; i <= 45; i++) {
        // Raiseleg1
        l0.s1.write(90);
        set_servo_forw(l1.s1, counters[1], 45);
        delay(ACTIONSPEED);

        // Move back to touch the ground2
        set_servo_forw(l2.s1, counters[2], 45);
        set_servo_back(l3.s1, counters[3], 90);
        delay(ACTIONSPEED);

        // Rise the leg3
        l4.s1.write(90);
        set_servo_forw(l5.s1, counters[5], 75);
        delay(ACTIONSPEED);

        // Move back to touch the ground4
        set_servo_back(l6.s1, counters[6], 135);
        set_servo_forw(l7.s1, i7H1, 90);
        delay(ACTIONSPEED);

        // Raise leg5
        l8.s1.write(90);
        set_servo_back(l9.s1, i9H1, 117.5);
        delay(ACTIONSPEED);

        // Move back to touch the ground6
        set_servo_back(l10.s1, i10H1, 135);
        set_servo_forw(l11.s1, i11H1, 45);
        delay(ACTIONSPEED);
    }
}


void move_forward_latest1() {
    /**********rotate forward and rotate backward to the initial position**********/
    // TODO: This is stupid. Fix it.
    //PROCESS 4:(raise the second leg)
    for (int i = 0; i <= 45; i++) {
        l0.s1.write(90);
        set_servo_back(l2.s1, i2H4, 90);
        delay(ACTIONSPEED);

        l4.s1.write(90);
        //delay(ACTIONSPEED);
        set_servo_forw(l6.s1, i6H4, 90);
        delay(ACTIONSPEED); // Why is this here?

        l8.s1.write(90);
        set_servo_forw(l10.s1, i10H4, 90);
        delay(ACTIONSPEED);

    }

    //PROCESS 3:
    /*(rorate second leg forward move second leg back to the ground) and
      (torate first leg backward)*/
    for (int i = 0; i <= 45; i++) {
        // Move back to touch the ground1
        l0.s1.write(90);//90
        set_servo_back(l1.s1, i1H3, 90);
        delay(ACTIONSPEED);

        // Raise leg2
        set_servo_forw(l2.s1, i2H3, 45);
        set_servo_forw(l3.s1, i3H3, 45);
        delay(ACTIONSPEED);

        // Move back to touch the ground3
        l4.s1.write(90);  //90
        set_servo_back(l5.s1, i5H3, 120);
        delay(ACTIONSPEED);
        // Rise the leg4
        set_servo_back(l6.s1, i6H3, 135);
        set_servo_back(l7.s1, i7H3, 135);
        delay(ACTIONSPEED);

        // Move back to touch the ground5
        l8.s1.write(90);  //90
        set_servo_forw(l9.s1, i9H3, 72.5);
        delay(ACTIONSPEED);

        // Raise leg6
        set_servo_back(l10.s1, i10H3, 135);
        set_servo_back(l11.s1, i11H3, 90);
        delay(ACTIONSPEED);

    }

    /*********rotate backward to the initial position and rotate forward**********/

    //PROCESS 5:(raise the first leg)
    for (int i = 0; i <= 45; i++){
        set_servo_back(l0.s1, i0H5, 90);
        l2.s1.write(90);
        delay(ACTIONSPEED);

        set_servo_back(l4.s1, i4H5, 90);
        l6.s1.write(90);
        delay(ACTIONSPEED);
        set_servo_forw(l8.s1, i8H5, 90);
        l10.s1.write(90);
        delay(ACTIONSPEED);

    }

    //PROCESS 1:
    /*(rorate first leg forward move first leg back to the ground) and
      (rorate second leg backward)*/
    for (int i = 0; i <= 45; i++) {
        // Rise the leg1
        set_servo_forw(l0.s1, i0H1, 45);
        set_servo_forw(l1.s1, i1H1, 45);

        delay(ACTIONSPEED);
        // Move back to touch the ground2
        l2.s1.write(90);
        set_servo_back(l3.s1, i3H1, 90);
        delay(ACTIONSPEED);
        // Raise leg3
        set_servo_forw(l4.s1, i4H1, 45);
        set_servo_forw(l5.s1, i5H1, 75);
        delay(ACTIONSPEED);

        // Move back to touch the ground4
        l6.s1.write(90);
        set_servo_forw(l7.s1, i7H1, 90);
        delay(ACTIONSPEED);

        // Raise leg5
        set_servo_back(l8.s1, i8H1, 135);
        set_servo_back(l9.s1, i9H1, 117.5);
        delay(ACTIONSPEED);

        // Move back to touch the ground6
        l10.s1.write(90);
        set_servo_forw(l11.s1, i11H1, 45);
        delay(ACTIONSPEED);
    }
}


void move_left_latest1() {
    // TODO: This is stupid. Fix it.
    //PROCESS 4:(raise the second leg)
    for (int i = 0; i <= 45; i++){
        l0.s1.write(90);
        set_servo_back(l2.s1, i2H4, 90);
        delay(ACTIONSPEED);

        l4.s1.write(90);
        set_servo_forw(l6.s1, i6H4, 90);

        delay(ACTIONSPEED);

        l8.s1.write(90);
        set_servo_forw(l10.s1, i10H4, 90);
        delay(ACTIONSPEED);
    }

    //PROCESS 3:
    /*(rorate second leg forward move second leg back to the ground) and
      (torate first leg backward)*/
    for (int i = 0; i <= 45; i++){
        // Move back to touch the ground1
        l0.s1.write(90);
        set_servo_forw(l1.s1, i1H3, 45);
        delay(ACTIONSPEED);

        // Raise leg2
        set_servo_forw(l2.s1, i2H3, 45);
        set_servo_back(l3.s1, i3H3, 90);
        delay(ACTIONSPEED);
        //  delay(1);

        // Move back to touch the ground3
        l4.s1.write(90);
        set_servo_forw(l5.s1, i5H3, 75);
        delay(ACTIONSPEED);
        // Rise the leg4
        set_servo_back(l6.s1, i6H3, 135);
        set_servo_back(l7.s1, i7H3, 135);
        delay(ACTIONSPEED);

        // Move back to touch the ground5
        l8.s1.write(90);
        set_servo_forw(l9.s1, i9H3, 72.5);
        delay(ACTIONSPEED);

        // Raise leg6
        set_servo_back(l10.s1, i10H3, 135);
        set_servo_back(l11.s1, i11H3, 90);
        delay(ACTIONSPEED);
    }

    /*********rotate backward to the initial position and rotate forward**********/


    //PROCESS 5:(raise the first leg)
    for (int i = 0; i <= 45; i++) {
        set_servo_back(l0.s1, i0H5, 90);
        l2.s1.write(90);
        delay(ACTIONSPEED);

        set_servo_back(l4.s1, i4H5, 90);
        l6.s1.write(90);
        delay(ACTIONSPEED);

        set_servo_forw(l8.s1, i8H5, 90);
        l10.s1.write(90);
        delay(ACTIONSPEED);
    }


    //PROCESS 1:
    /*(rorate first leg forward move first leg back to the ground) and
      (torate second leg backward)*/
    for (int i = 0; i <= 45; i++) {
        // Rise the leg1
        set_servo_forw(l0.s1, i0H1, 45);
        set_servo_back(l1.s1, i1H1, 90);
        delay(ACTIONSPEED);

        // Move back to touch the ground2
        l2.s1.write(90);
        set_servo_forw(l3.s1, i3H1, 45);
        delay(ACTIONSPEED);

        // Raise leg3
        set_servo_forw(l4.s1, i4H1, 45);
        set_servo_back(l5.s1, i5H1, 120);
        delay(ACTIONSPEED);

        // Move back to touch the ground4
        l6.s1.write(90);
        set_servo_forw(l7.s1, i7H1, 90);
        delay(ACTIONSPEED);

        // Rise the leg5
        set_servo_back(l8.s1, i8H1, 135);
        set_servo_back(l9.s1, i9H1, 117.5);
        delay(ACTIONSPEED);

        // Move back to touch the ground6
        l10.s1.write(90);
        set_servo_forw(l11.s1, i11H1, 45);
        delay(ACTIONSPEED);
    }
}


void move_right_latest1() {
/**********rotate forward and rotate backward to the initial position**********/
    // TODO: Fix this.

    //PROCESS 4:(raise the second leg)

    for (int i = 0; i <= 45; i++) {
        // TODO: Set this inside another loop.
        l0.s1.write(90);
        set_servo_back(l2.s1, i2H4, 90);
        delay(ACTIONSPEED);

        l4.s1.write(90);
        set_servo_forw(l6.s1, i6H4, 90);
        delay(ACTIONSPEED);

        l8.s1.write(90);
        set_servo_forw(l10.s1, i10H4, 90);
        delay(ACTIONSPEED);
    }

    //PROCESS 3:
    /*(rorate second leg forward move second leg back to the ground) and
      (torate first leg backward)*/
    for (int i = 0; i <= 45; i++){
        // Move back to touch the ground1
        l0.s1.write(90);
        set_servo_back(l1.s1, i1H3, 90);
        delay(ACTIONSPEED);

        // Raise leg2
        set_servo_forw(l2.s1, i2H3, 45);
        set_servo_forw(l3.s1, i3H3, 45);
        delay(ACTIONSPEED);

        // Move back to touch the ground3
        l4.s1.write(90);
        set_servo_back(l5.s1, i5H3, 120);
        delay(ACTIONSPEED);

        // Raise leg4
        set_servo_back(l6.s1, i6H3, 135);
        set_servo_forw(l7.s1, i7H3, 90);
        delay(ACTIONSPEED);

        // Move back to touch the ground5
        l8.s1.write(90);
        set_servo_back(l9.s1, i9H3, 117.5);
        delay(ACTIONSPEED);

        // Raise leg6
        set_servo_back(l10.s1, i10H3, 135);
        set_servo_forw(l11.s1, i11H3, 45);
        delay(ACTIONSPEED);
    }

    /*********rotate backward to the initial position and rotate forward**********/


    //PROCESS 5:(raise the first leg)
    for (int i = 0; i <= 45; i++) {
        // TODO: Get this inside another loop.
        set_servo_back(l0.s1, i0H5, 90);
        l2.s1.write(90);
        delay(ACTIONSPEED);

        set_servo_back(l4.s1, i4H5, 90);
        l6.s1.write(90);
        delay(ACTIONSPEED);

        set_servo_forw(l8.s1, i8H5, 90);
        l10.s1.write(90);
        delay(ACTIONSPEED);
    }

    //PROCESS 1:
    /*(rorate first leg forward move first leg back to the ground) and
      (torate second leg backward)*/
    for (int i = 0; i <= 45; i++) {
        // Rise the leg1
        set_servo_forw(l0.s1, i0H1, 45);
        set_servo_forw(l1.s1, i1H1, 45);
        delay(ACTIONSPEED);

        // Move back to touch the ground2
        l2.s1.write(90);
        set_servo_back(l3.s1, i3H1, 90);
        delay(ACTIONSPEED);

        // Raise leg3
        set_servo_forw(l4.s1, i4H1, 45);
        set_servo_forw(l5.s1, i5H1, 75);
        delay(ACTIONSPEED);

        // Move back to touch the ground4
        l6.s1.write(90);
        set_servo_back(l7.s1, i7H1, 135);
        delay(ACTIONSPEED);

        // Raise leg5
        set_servo_forw(l9.s1, i9H1, 72.5);
        set_servo_back(l8.s1, i8H1, 135);
        delay(ACTIONSPEED);

        // Move back to touch the ground6
        l10.s1.write(90);
        set_servo_back(l11.s1, i11H1, 90);
        delay(ACTIONSPEED);
    }
    /**********rotate forward and rotate backward to the initial position**********/
}


void ESP8266_ATCOMMAND() {
    Serial.begin(115200);       // set up a wifi serial communication baud rate 115200
    //   pinMode(controller, OUTPUT);    // sets the RelayPin to be an output


    Serial.println("AT+CWMODE=3\r\n");//set to softAP+station mode
    delay(2000);     //delay 4s

    Serial.println("AT+CWSAP=\"Adeept_ESP\",\"12345678\",8,2\r\n");   //TCP Protocol, server IP addr, port
    delay(3000);     //delay 4s
    Serial.println("AT+RST\r\n");     //reset wifi
    delay(3000);     //delay 4s



    /* Serial.println("AT+CWJAP=\"Adeeptesp8266\",\"Adeept1315\"\r\n");   //replace "MERCURY_BE90" to your WiFi AP name(SSID), replace "adeept1205" to your WiFi password
      delay(4000);     //delay 4s
      delay(4000);     //delay 4s
      Serial.println("AT+CIPSTART=\"TCP\",\"192.168.4.1\",1215\r\n");   //TCP Protocol, server IP addr, port
      delay(4000);     //delay 4s
      delay(4000);     //delay 4s
    */
    Serial.println("AT+CIPMUX=1\r\n");
    delay(3000);
    Serial.println("AT+CIPSERVER=1\r\n");
    delay(3000);
    Serial.println("AT+CIPSTO=7000\r\n");
    delay(3000);
}


void self_balanced_setup() {
    Wire.begin();                                                        //Start I2C as master
    Serial.begin(115200);                                               //Use only for debugging
    pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output

    setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

    digitalWrite(13, HIGH);                                              //Set digital output 13 high to indicate startup

    for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                 //Run this code 2000 times
        //    if(cal_int % 125 == 0)Serial.print(".");                           //Print a dot on the LCD every 125 readings
        read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
        gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
        gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
        gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
        delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
    }
    gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
    gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
    gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset

    digitalWrite(13, LOW);                                               //All done, turn the LED off

    loop_timer = micros();                                               //Reset the loop timer
}



void self_balanced() {
    read_mpu_6050_data();                                                //Read the raw acc and gyro data from the MPU-6050

    gyro_x -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
    gyro_y -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
    gyro_z -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
    angle_roll += gyro_y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
    //  angle_yaw += gyro_z * 0.0000611;

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
    angle_pitch += angle_roll * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
    angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel

    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); //Calculate the total accelerometer vector
    //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;    //Calculate the pitch angle
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;    //Calculate the roll angle

    //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
    angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
    angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

    if (set_gyro_angles) {                                               //If the IMU is already started
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
    }
    else {                                                               //At first start
        angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle
        angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle
        set_gyro_angles = true;                                            //Set the IMU started flag
    }

    //To dampen the pitch and roll angles a complementary filter is used
    angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
    angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
    angle_yaw_output = angle_yaw_output * 0.9 + angle_yaw * 0.1;

    //angle_pitch_output = angle_pitch * 0.8;
    //angle_roll_output = angle_roll * 0.90+counter*0.00003333;
    //angle_yaw_output = angle_yaw * 0.635+counter*0.000008;

    //  Serial.print(angle_roll_output);
    //  Serial.print("  ");
    //  Serial.println(angle_pitch_output);
    //counter++;
    while (micros() - loop_timer < 4000);                                //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
    loop_timer = micros();                                               //Reset the loop timer
}


void read_mpu_6050_data() {                                            //Subroutine for reading the raw gyro and accelerometer data
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x3B);                                                    //Send the requested starting register
    Wire.endTransmission();                                              //End the transmission
    Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
    //  while(Wire.available() < 14);                                        //Wait until all the bytes are received
    delay(1);
    acc_x = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_x variable
    acc_y = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_y variable
    acc_z = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_z variable
    temperature = Wire.read() << 8 | Wire.read();                        //Add the low and high byte to the temperature variable
    gyro_x = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_x variable
    gyro_y = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_y variable
    gyro_z = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_z variable

}


void setup_mpu_6050_registers() {

    //Activate the MPU-6050
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x6B);                                                    //Send the requested starting register
    Wire.write(0x00);                                                    //Set the requested starting register
    Wire.endTransmission();                                              //End the transmission
    //Configure the accelerometer (+/-8g)
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x1C);                                                    //Send the requested starting register
    Wire.write(0x10);                                                    //Set the requested starting register
    Wire.endTransmission();                                              //End the transmission
    //Configure the gyro (500dps full scale)
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x1B);                                                    //Send the requested starting register
    Wire.write(0x08);                                                    //Set the requested starting register
    Wire.endTransmission();                                              //End the transmission
}


void Stand_Up() {
    // Stand upright
    l0.s1.write(90);
    l1.s1.write(90); //45
    l3.s1.write(90);//90
    l2.s1.write(90); //10
    l4.s1.write(90);
    l5.s1.write(90); //120
    l6.s1.write(90);//180
    l7.s1.write(90); //90
    l8.s1.write(90);
    l9.s1.write(95); //95
    l11.s1.write(90);//0
    l10.s1.write(90);   //180
    l12.s1.write(90);
}


void self_balanced_test() {
    //left-leaning
    if (angle_pitch_output < -3 && angle_roll_output < 5 && angle_roll_output > -5) {
        l0.s1.write(90 + angle_pitch_output * 2 * 2 / 12);
        l2.s1.write(90 + angle_pitch_output * 2 * 2 / 12);
        l4.s1.write(90 + angle_pitch_output * 2 * 2 / 12);
        l6.s1.write(90 - angle_pitch_output * 2 * 10 / 12);
        l8.s1.write(90 - angle_pitch_output * 2 * 10 / 12);
        l10.s1.write(90 - angle_pitch_output * 2 * 10 / 12);

    }
    //right-leaning
    if (angle_pitch_output > 3 && angle_roll_output < 5 && angle_roll_output > -5) {
        l0.s1.write(90 - angle_pitch_output * 2 * 10 / 12);
        l2.s1.write(90 - angle_pitch_output * 2 * 10 / 12);
        l4.s1.write(90 - angle_pitch_output * 2 * 10 / 12);
        l6.s1.write(90 + angle_pitch_output * 2 * 2 / 12);
        l8.s1.write(90 + angle_pitch_output * 2 * 2 / 12);
        l10.s1.write(90 + angle_pitch_output * 2 * 2 / 12);

    }
    //forward-leaning
    if (angle_roll_output > 3 && angle_pitch_output < 3 && angle_pitch_output > -3) {
        l0.s1.write(90 - angle_roll_output * 5 * 1 / 12);
        l6.s1.write(90 + angle_roll_output * 5 * 1 / 12);
        l2.s1.write(90 - angle_roll_output * 5 * 8 / 12);
        l8.s1.write(90 + angle_roll_output * 5 * 8 / 12);
        l4.s1.write(90 - angle_roll_output * 5 * 9 / 12);
        l10.s1.write(90 + angle_roll_output * 5 * 9 / 12);

    }
    //backward-leaning
    if (angle_roll_output < -3 && angle_pitch_output < 3 && angle_pitch_output > -3) {
        l0.s1.write(90 + angle_roll_output * 5 * 9 / 12);
        l6.s1.write(90 - angle_roll_output * 5 * 9 / 12);
        l2.s1.write(90 + angle_roll_output * 5 * 8 / 12);
        l8.s1.write(90 - angle_roll_output * 5 * 8 / 12);
        l4.s1.write(90 + angle_roll_output * 5 * 1 / 12);
        l10.s1.write(90 - angle_roll_output * 5 * 1 / 12);


    }
}

long ultrasonic() {
    // TODO: Fix this
    long a = sr04.Distance();
    if (a > 100) {
        a = 0;
    }
    return a;
    //  if(angel%3 == 0){
    //  u[]=a;
    //  }

    //  Serial.print(a);
    //  Serial.println("cm");
}

void set_servo_back(Servo& servo, int &position, double value){
    servo.write(value - position);
    position++;
    position = position % 46;
}

void set_servo_forw(Servo& servo, int &position, double value){
    servo.write(value + position);
    position++;
    position = position % 46;
}

void step_by_step() {
    /* Recoded with a temporary solution until I understand the code better and can make something nicer. This is to save flash memory*/
    Servo* servos_loop1[7] = {&l1.s1, &l2.s1, &l4.s1, &l6.s1, &l8.s1, &l10.s1, &l1.s1};// &l1.s1, &l3.s1, &l5.s1, &l7.s1, &l9.s1, &l11.s1};
    int* counters_loop1[7] = {&i0H0, &i2H0, &i4H0, &i6H0, &i8H0, &i10H0, &i1H0};
    for (int i = 0; i <= 45; i++){
        for (int k = 0; k < 7; k++){
            set_servo_back(*(servos_loop1[k]), *(counters_loop1[k]), 135);
            delay(ACTIONSPEED);
        }
        set_servo_back(l1.s1, i1H0, 45);
        delay(ACTIONSPEED);
        set_servo_back(l3.s1, i3H0, 45);
        delay(ACTIONSPEED);
        set_servo_back(l5.s1, i5H0, 120);
        delay(ACTIONSPEED);
        set_servo_forw(l7.s1, i7H0, 135);
        delay(ACTIONSPEED);
        set_servo_forw(l9.s1, i9H0, 95);
        delay(ACTIONSPEED);
        set_servo_forw(l11.s1, i11H0, 45);
        delay(ACTIONSPEED);

        if (Serial.available() > 0) {
            break;
        }
        delay(ACTIONSPEED);
    }

    Servo* Servos_loop2[] = {&l0.s1, &l2.s1, &l4.s1, &l6.s1, &l8.s1, &l10.s1};
    int* counters_loop2[] = {&i0H0, &i2H0, &i4H0, &i6H0, &i8H0, &i10H0};
    for (int i = 0; i <= 45; i++){
        for (int k = 0; k < 6; k++){
            set_servo_back(*(Servos_loop2[k]), *(counters_loop2[k]), 90);
            delay(ACTIONSPEED);
        }
        set_servo_forw(l1.s1, i1H0, 0);
        delay(ACTIONSPEED);
        set_servo_forw(l3.s1, i3H0, 0);
        delay(ACTIONSPEED);
        set_servo_forw(l5.s1, i5H0, 75);
        delay(ACTIONSPEED);
        set_servo_back(l7.s1, i7H0, 180);
        delay(ACTIONSPEED);
        set_servo_back(l9.s1, i9H0, 135);
        delay(ACTIONSPEED);
        set_servo_back(l11.s1, i11H0, 90);
        delay(ACTIONSPEED);

        if (Serial.available() > 0) {
            break;
        }
        delay(ACTIONSPEED);
    }
    Servo* Servos_loop3[8] = {&l0.s1, &l2.s1, &l4.s1, &l6.s1, &l8.s1, &l10.s1, &l1.s1, &l3.s1};
    int* counters_loop3[8] = {&i0H0, &i2H0, &i4H0, &i6H0, &i8H0, &i10H0, &i11H0, &i3H0};
    for (int i = 0; i <= 45; i++){
        for (int k = 0; k < 8; k++){
            set_servo_forw(*(Servos_loop3)[k], *(counters_loop3[k]), 45);
            delay(ACTIONSPEED);
        }
        set_servo_forw(l5.s1, i5H0, 120);
        delay(ACTIONSPEED);
        set_servo_back(l7.s1, i7H0, 135);
        delay(ACTIONSPEED);
        set_servo_back(l9.s1, i9H0, 95);
        delay(ACTIONSPEED);
        set_servo_back(l11.s1, i11H0, 45);
        delay(ACTIONSPEED);

        if (Serial.available() > 0) {
            break;
        }
        delay(ACTIONSPEED);
    }
    Servo* Servos_loop4[6] = {&l0.s1, &l2.s1, &l4.s1, &l6.s1, &l8.s1, &l10.s1};
    int* counters_loop4[6] = {&i0H0, &i2H0, &i4H0, &i6H0, &i8H0, &i10H0};
    for (int i = 0; i <= 45; i++){
        for (int k = 0; k < 6; k++){
            // +90
            set_servo_forw(*(Servos_loop4[k]), *(counters_loop4[k]), 90);
            delay(ACTIONSPEED);
        }
        set_servo_back(l1.s1, i1H0, 90);
        delay(ACTIONSPEED);
        set_servo_back(l3.s1, i3H0, 90);
        delay(ACTIONSPEED);
        set_servo_back(l5.s1, i5H0, 165);
        delay(ACTIONSPEED);
        set_servo_forw(l7.s1, i7H0, 90);
        delay(ACTIONSPEED);
        set_servo_forw(l9.s1, i9H0, 50);
        delay(ACTIONSPEED);
        set_servo_forw(l11.s1, i11H0, 0);
        delay(ACTIONSPEED);

        if (Serial.available() > 0) {
            break;
        }
        delay(ACTIONSPEED);
    }
    battery();
    sendbattery();
}

void doWhatever(int val1, int& val2, bool add){
    // What is this for?
    for (int i = 0; i <= 30; i++){
        delay(10);
        if (add){
            l12.s1.write(val1 + val2);
        }
        else{
            l12.s1.write(val1 - val2);
        }
        val2++;
        if (val2 == 31){
            val2 = 0;
        }
        if (Serial.available() > 0){
            break;
        }
    }
}


void attack() {

    long a = ultrasonic();
    if (a <= 15 && a > 0) {
        l0.s1.write(90);
        l1.s1.write(0); //45
        l3.s1.write(0);//90
        l2.s1.write(45); //10
        l4.s1.write(45);
        l5.s1.write(75); //120
        l6.s1.write(90);//180
        l7.s1.write(180); //90
        l8.s1.write(135);
        l9.s1.write(135); //95
        l11.s1.write(90);//0
        l10.s1.write(135);   //180

        doWhatever(90, i2H4, true);
        doWhatever(120, i2H4, false);
        doWhatever(90, i2H4, false);
        doWhatever(60, i2H4, true);
    }
    else if (a > 15) {
        while (a > 15) {
            ultrasonic();
            original_latest();
            if (Serial.available() > 0) {
                break;
            }
        }
    }
}

char judgement(String &comdata) { // uses serial input to determine our next action
    // TODO: See if we can reprogam the control script so this can be simplified.
    char judge = 0;
    if (comdata.length() > 0)
    { if (comdata.endsWith(text1) || comdata.endsWith(phone1)) { //forward
            judge = 1;
        }
        if (comdata.endsWith(text2) || comdata.endsWith(phone2)) { //backward
            judge = 2;
        }
        if (comdata.endsWith(text3) || comdata.endsWith(phone3)) { //right
            judge = 3;
        }
        if (comdata.endsWith(text4) || comdata.endsWith(phone4)) { //left
            judge = 4;
        }
        if (comdata.endsWith(text5) || comdata.endsWith(phone5)) { //balance
            judge = 5;
        }
        if (comdata.endsWith(text6) || comdata.endsWith(phone6)) { //aviod
            judge = 6;
        }
        if (comdata.endsWith(text7))
        {
            judge = 7;
        }
        if (comdata.endsWith(phone7) || comdata.endsWith(phone8) || comdata.endsWith(phone9) || comdata.endsWith(phone10)) {
            judge = 11; //Starting position
        }
        if (comdata == text8) {
            judge = 8;
        }
        if (comdata.endsWith(text9)) {
            judge = 9;
        }
        if (comdata.endsWith(text5))
        {
            ;//A
        }
        if (comdata.endsWith(text6))
        {
            ;//B
        }
        comdata = "";
        delay(10); // varför??
    }
    return judge;
}

void advoid() { // Called when user pushes "Auto". Im assuming its going to walk around and try to avoid things in front of it
    /**********rotate forward and rotate backward to the initial position**********/

    //PROCESS 4:(raise the second leg)
    int cntr = 0;
    for (int i = 0; i <= 45; i++){
        l0.s1.write(90);
        set_servo_back(l2.s1, i2H4, 100);
        delay(ACTIONSPEED);
        l4.s1.write(90);
        set_servo_forw(l6.s1, i6H4, 90);
        delay(ACTIONSPEED);
        l8.s1.write(90);
        set_servo_forw(l10.s1, i10H4, 90);
        delay(ACTIONSPEED);
        l12.s1.write(90 + i12H4);

        if (cntr < 20) { // TODO: Does this need to be here?
            cntr++;
        }

        if (Serial.available() > 0) {
            break;
        }
    }


    //PROCESS 3:
    /*(rorate second leg forward move second leg back to the ground) and
      (torate first leg backward)*/
    cntr = 0;
    for (int i = 0; i <= 45; i++){
        // Move back to touch the ground1
        l0.s1.write(90);//90
        set_servo_back(l1.s1, i1H3, 90);
        delay(ACTIONSPEED);
        // Rise the leg2
        set_servo_forw(l2.s1, i2H3, 45);
        set_servo_forw(l3.s1, i3H3, 22.5);
        delay(ACTIONSPEED);

        // Move back to touch the ground3
        l4.s1.write(90);  //90
        set_servo_back(l5.s1, i5H3, 120);
        delay(ACTIONSPEED);

        // Raise leg4
        set_servo_back(l6.s1, i6H3, 135);
        set_servo_back(l7.s1, i7H3, 135);
        delay(ACTIONSPEED);

        // Move back to touch the ground5
        l8.s1.write(90);  //90
        set_servo_forw(l9.s1, i9H3, 72.5);
        delay(ACTIONSPEED);

        // Raise leg6
        set_servo_back(l10.s1, i10H3, 135);
        set_servo_back(l11.s1, i11H3, 90);
        delay(ACTIONSPEED);
        l12.s1.write(110 - cntr);

        if (cntr < 20) {
            cntr++;
        }


        if (Serial.available() > 0) {
            break;
        }
        //  delay(ACTIONSPEED);
    }
    /*********rotate backward to the initial position and rotate forward**********/

    //PROCESS 5:(raise the first leg)
    cntr = 0;
    for (int i = 0; i <= 45; i++) {
        set_servo_back(l0.s1, i0H5, 90);
        l2.s1.write(90);
        delay(ACTIONSPEED);

        set_servo_back(l4.s1, i4H5, 90);
        l6.s1.write(90);
        delay(ACTIONSPEED);

        set_servo_forw(l8.s1, i8H5, 90);
        l10.s1.write(90);
        delay(ACTIONSPEED);

        l12.s1.write(90 - i12H5);

        if (cntr < 20) {
            cntr++;
        }

        if (Serial.available() > 0) {
            break;
        }
    }

    for (int i = 0; i < 6; i++) {
        strip.setPixelColor(i, strip.Color(129, 67, 213));
        strip.show();
        delay(1);
    }
    //PROCESS 1:
    /*(rorate first leg forward move first leg back to the ground) and
      (rorate second leg backward)*/
    cntr = 0;
    for (int i = 0; i <= 45; i++){
        // Raise leg1
        set_servo_forw(l0.s1, i0H1, 45);
        set_servo_forw(l1.s1, i1H1, 45);
        delay(ACTIONSPEED);
        // Move back to touch the ground2
        l2.s1.write(90);
        set_servo_back(l3.s1, i3H1, 67.5);
        delay(ACTIONSPEED);

        // Raise leg3
        set_servo_forw(l4.s1, i4H1, 45);
        set_servo_forw(l5.s1, i5H1, 75);
        delay(ACTIONSPEED);

        // Move back to touch the ground4
        l6.s1.write(90);
        set_servo_forw(l7.s1, i7H1, 90);
        delay(ACTIONSPEED);

        // Rise the leg5
        set_servo_back(l8.s1, i8H1, 135);
        set_servo_back(l9.s1, i9H1, 117.5);
        delay(ACTIONSPEED);

        // Move back to touch the ground6
        l10.s1.write(90);
        set_servo_forw(l11.s1, i11H1, 45);
        delay(ACTIONSPEED);

        l12.s1.write(70 + i12H1);

        if (cntr < 20) {
            cntr++;
        }

        if (Serial.available() > 0) {
            break;
        }
    }

    long a = ultrasonic();
    if (a <= 20 && a > 0) {
        for (int i = 0; i < 6; i++){
            move_right_latest1();
            if (Serial.available() > 0) {
                break;
            }
        }
    }
    //  battery();

    //  sendbattery();

}