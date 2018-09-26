// pololu-tracker.cpp - GitHub repository at https://github.com/Awesomelotad/pololu-tracker if needed
// Developed by: Brandon Ash Lovegrove, ID: 17982686
// ENSE504 2018 Assignment 2 program code

#include "mbed.h"
#include "m3pimaze.h"
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <map>

using namespace std;

// define the m3pi (robot) and Serial (wixel) objects
m3pi robot(p23, p9, p10);
Serial wixel(p28, p27);

// create timer
Timer timer;

// --------------------------------- variable/const. pre-alloc -----------------------------------

// define global variables
float motorSpeedMax = 0.2; // CHANGE MAXIMUM ROBOT SPEED HERE (slower == better)
float runTime_s = 0.0; // time robot has been running in seconds

// --------------------------------------- custom functions ---------------------------------------

// updates motor speeds from 'motorSpeeds' array with ease
// args: same as definition in main()
int updateMotors(float motorSpeeds[]) {

    robot.left_motor(motorSpeeds[1]);
    robot.right_motor(motorSpeeds[0]);
    
    return 0;
}

// ------------------------------------------- primary function -------------------------------------------

int main() {

    // define local variables
    int tNext_ms = 100; // time until next transmission to PC in milliseconds (100ms interval between transmit because 50 is insane)
    int sensors[5]; // array for light sensor values
    int transmitResult; // result given from transmitData() function
    float trackProx = 0.0; // relative position of robot to track
    float motorSpeeds[] = {motorSpeedMax, motorSpeedMax}; // left and right motor speeds respectively (reversed for m3pimaze fault)
    bool onTrack = true; // true/false if robot is on/off the line ('track')
   
    // set the Wixel communication speed (baud rate / bitrate)
    wixel.baud(115200);

    // display the program title on the PC
    wixel.printf("Pololu Tracker\r\n\r\n");

    // get and display the battery voltage
    wixel.printf("Battery voltage: %.3f\r\n\r\n", robot.battery());

    // display the PC column headings / key
    wixel.printf("Robot data log key:\r\nTIM = time since robot started\r\nPRX = Robot proximity to track\r\nSN-X = Where X is the light sensor ID from left to right / 0 to 5\r\n\r\n");
    wixel.printf(" ______________________________________________________\r\n");
    wixel.printf("|   TIM   |   PRX   | SN-0 | SN-1 | SN-2 | SN-3 | SN-4 |\r\n"); // print formatted column-headers
    wixel.printf("|`````````|`````````|``````|``````|``````|``````|``````|\r\n"); // print blank spacer line

    // display the 'active' message on robot LCD on boot (+style animation)
    robot.cls();
    wait(.5);
    robot.locate(0, 0);
    robot.printf("Loading!");
    robot.locate(0, 1);
    robot.printf("[      ]");
    for (int i=1; i <= 6; i++) {
        robot.locate(i, 1);
        robot.printf("=");
        wait(.1);
    }
    for (int i=1; i <= 5; i++) {
        robot.locate(0, 0);
        robot.printf(" !Done! ");
        wait(.2);
        robot.locate(0, 0);
        robot.printf("        ");
        wait(.2);
    }
    robot.locate(0, 0);
    robot.printf("Tracking");
    robot.locate(0, 1);
    robot.printf("        ");
    wait(.5);
    
    // calibrate the sensors to find the line
    robot.sensor_auto_calibrate();

    wait(.5);

    // start robot motors
    updateMotors(motorSpeeds);

    // start the timer
    timer.reset();
    timer.start();
    
    // the main loop
    while(1) {

        // run while robot is following the track (calibration at init. means default state is always true)
        do {

            robot.readsensor(sensors); // read and set sensor data

            // determine if robot is still on the track
            if (sensors[1] >= 600 || sensors[2] >=600 || sensors[3] >= 600) {
                onTrack = true;
            } else {
                onTrack = false;
            }

            // calculate robot's centerline proximity to the track for motor control
            // assesment says this needs to be a function but it is only used once - making it function is irrational.
            trackProx = (float)(sensors[3] - sensors[1]) / (sensors[1] + sensors[2] + sensors[3]);

            if (trackProx >= 0) {
                motorSpeeds[0] = motorSpeedMax;
                motorSpeeds[1] = motorSpeedMax * (1 - trackProx);
            } else if (trackProx < 0) {
                motorSpeeds[1] = motorSpeedMax;
                motorSpeeds[0] = motorSpeedMax * (1 + trackProx);
            }

            // update motor speed
            updateMotors(motorSpeeds);

            // check and update timer/transmit data
            int tNow_ms = timer.read_ms(); // get current time
            timer.reset();
            runTime_s += tNow_ms * 0.001; // update current running time
            if (tNext_ms <= 0) {
            // transmit robot data to PC
                wixel.printf("|%-9.2f|%-9.2f|%-6d|%-6d|%-6d|%-6d|%-6d|\r\n", runTime_s, trackProx, sensors[1], sensors[2], sensors[3]);
                tNext_ms = 100; // reset time to next transmission
            } else {
                tNext_ms -= tNow_ms; // amount to reduce timer 
            }

        } while (onTrack == true);

        // transmit last position of robot goes off track and terminate program
        int tNow_ms = timer.read_ms(); // get current time
        runTime_s += tNow_ms * 0.001; // update current running time
        wixel.printf("|%-9.2f|%-9.2f|%-6d|%-6d|%-6d|%-6d|%-6d|\r\n", runTime_s, trackProx, sensors[1], sensors[2], sensors[3]);
        robot.stop();
        break;
        // note: this will only trigger if the track is not a closed loop

    }
}


