/**
 ********************************************************************************************
 * @file    PID_tuning.cpp
 * @brief   Program to tune the PID gains
 * @details 
 ********************************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 ********************************************************************************************
 */

#include "KMR_CubeMars.hpp"
#include "unistd.h"
#include <cmath>

#include <iostream>
#include <sstream> 
#include <fstream> 


using namespace std;

// --------------------------------------------------------------------------- //
//                                EDIT HERE 
// --------------------------------------------------------------------------- //

// Id(s) and model(s) of motor(s)
vector<int> ids = {1}; 
int nbrMotors = ids.size();
vector<KMR::CBM::Model> models{KMR::CBM::Model::AK60_6};

// CAN bus
const char* can_bus = "can0";

// Kp and Kd (!! highly recommended to start with low values of Kp)
#define Kp  150
#define Kd  1

// Control period
#define CTRL_PERIOD_US  5000  // 5ms => 200 Hz
// --------------------------------------------------------------------------- //

#define MAX_CTR         1000

#define MAX_SPEED       2*M_PI/1  // rad/s
#define AVG_IMPULSE     MAX_SPEED*CTRL_PERIOD_US/1000000.0

#define SPEED1          2*M_PI/1  // rad/s
#define SPEED2          2*M_PI/3  // rad/s
#define INCREMENT       SPEED1*CTRL_PERIOD_US/1000000.0
#define INCREMENT2      SPEED2*CTRL_PERIOD_US/1000000.0


// Stream
std::ofstream logStream, trackingStream;

// Idea: 200 Hz control loop -> 5 ms period
// Speed = 2PI in 1 sec
// -> Increment per loop = 2PI * 5ms = 0.03 rad

int main()
{
    KMR::CBM::MotorHandler motorHandler(ids, can_bus, models);

    // Set Kp and Kd
    vector<float> Kps(nbrMotors, Kp);
    vector<float> Kds(nbrMotors, Kd);
    motorHandler.setKps(ids, Kps);
    motorHandler.setKds(ids, Kds);
    
    // Enable motor
    vector<float> goalPositions(nbrMotors, 0);
    vector<float> fbckPositions(nbrMotors), fbckSpeeds(nbrMotors), fbckTorques(nbrMotors);
    vector<int> fbckTemperatures(nbrMotors);
    sleep(1);

    // Set the 0 reference
    cout << "Setting 0" << endl;
    motorHandler.setZeroPosition();
    sleep(1);

    // Set vars
    float time = 0;
    int ctr = 0, overtimeCtr = 0;
    float angle = 0;
    bool forward = 1;


    // =======================================================================
    //                First part: response to a typical impulse
    // =======================================================================

    // Main loop
    cout << endl << endl << " ---------- Starting impulses ---------" << endl;
    logStream.open ("../python_scripts/PID_tuning.csv");
    sleep(1);

    float impulse = 0;

    // Main loop
    while (ctr < MAX_CTR) {

        timespec start = KMR::CBM::time_s();

        // Get new goal position
        if (ctr == 200)
            impulse = AVG_IMPULSE;
        else if (ctr == 600)
            impulse = 0;

        // Get time
        time = ctr * CTRL_PERIOD_US/1000000.0;

        // Get feedback
        if (ctr == 0)
            motorHandler.getPositions(fbckPositions, 0);
        else    
            motorHandler.getPositions(fbckPositions);

        // Save time, goal position, fbck position, Kp, Kd
        logStream << time << "," << impulse << "," << fbckPositions[0] << "," << Kp << "," << Kd << endl;    

        // Send new goal positions
        for (int i=0; i<nbrMotors; i++)
            goalPositions[i] = impulse;

        motorHandler.setPositions(goalPositions);

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = KMR::CBM::time_s();
        double elapsed = KMR::CBM::get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0) {
            toSleep_us = 0;
            //cout << "Overtime at step " << ctr << " , elapsed = " << elapsed << " us" << endl;
        }

        usleep(toSleep_us);
    }

    logStream.close();

    sleep(1);

    // =======================================================================
    //                  Second part: tracking a full trajectory
    // =======================================================================

    // Reset vars
    time = 0;
    ctr = 0, overtimeCtr = 0;
    angle = 0;
    forward = 1;

    trackingStream.open("../python_scripts/tracking.csv");

    // ----- Starting tracking ----- //
    cout << "Starting trajectory tracking" << endl;
    sleep(1);
    timespec startTracking = KMR::CBM::time_s();
    while (ctr < MAX_CTR) {

        timespec start = KMR::CBM::time_s();

        // Get feedback
        if (ctr == 0)
            motorHandler.getPositions(fbckPositions, 0);
        else    
            motorHandler.getPositions(fbckPositions);

        // Get current time
        timespec now = KMR::CBM::time_s();
        double elapsedTracking = KMR::CBM::get_delta_us(now, startTracking); 
        time = elapsedTracking/1000000.0;

        // Save time, goal angle, fbck position, Kp, Kd
        trackingStream << time << "," << angle << "," << fbckPositions[0] << "," << Kp << "," << Kd << endl;  

        // Send new goal positions
        for (int i=0; i<nbrMotors; i++)
            goalPositions[i] = angle;
        motorHandler.setPositions(goalPositions);

        // Update the goal angle for next loop
        if (forward) {
            angle += INCREMENT;

            if (angle > M_PI) {
                angle = M_PI;
                forward = false;
            }
        }
        else {
            angle -= INCREMENT2;

            if (angle < -M_PI) {
                angle = -M_PI;
                forward = true;
            }
        }

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = KMR::CBM::time_s();
        double elapsed = KMR::CBM::get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0) {
            toSleep_us = 0;
            //cout << "Overtime at step " << ctr << " , elapsed = " << elapsed << " us" << endl;
            overtimeCtr++;
        }

        usleep(toSleep_us);
    }   

    trackingStream.close();

    cout << endl << endl << "The PID tuning successfully finished." << endl;
    cout << endl;
    cout << "Loops longer than the control period due to scheduling delays: " << overtimeCtr <<
            "/" << ctr << " (" << (float)overtimeCtr/(float)ctr*100.0 << "%)" << endl; 

    motorHandler.getTemperatures(fbckTemperatures, 0);
    cout << endl;
    for (int i=0; i<nbrMotors; i++)
        cout << "Motor " << ids[i] << "'s temperature is " << fbckTemperatures[i] << " °C" << endl;

    return(1);
}