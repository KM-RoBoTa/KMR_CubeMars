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



#define ENCODER_BIT_RESOLUTION  14
#define ANG_RESOLUTION          2*M_PI/pow(2, ENCODER_BIT_RESOLUTION)
#define FACTOR                  50 
#define MIN_IMPULSE             FACTOR*ANG_RESOLUTION
//#define MIN_IMPULSE             0.03
//#define MIN_IMPULSE             1.57

#define CTRL_PERIOD_US  5000  // 5ms = 200 Hz
#define MAX_CTR         1000

#define MAX_SPEED       2*M_PI/1  // rad/s
#define AVG_IMPULSE     MAX_SPEED*CTRL_PERIOD_US/1000000.0

#define Kp  150
#define Kd  1


// Stream
std::ofstream logStream;

// Idea: 200 Hz control loop -> 5 ms period
// Speed = 2PI in 1 sec
// -> Increment per loop = 2PI * 5ms = 0.03 rad


using namespace std;

int main()
{
    logStream.open ("../python_scripts/PID_tuning.csv");

    cout << "Min impulse = " << MIN_IMPULSE << " rad" << endl;

    float impulse = 0;

    vector<int> ids(1,1);
    int nbrMotors = ids.size();
    vector<KMR::CBM::Model> models(nbrMotors, KMR::CBM::Model::AK60_6);

    const char* can_bus = "can0";
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
    cout << "Going to MIT mode" << endl;
    motorHandler.enableMotors();
    sleep(1);

    // Set the 0 reference
    cout << "Setting 0" << endl;
    motorHandler.setZeroPosition();
    sleep(1);

    // Main loop
    cout << endl << endl << " ---------- Starting impulses ---------" << endl;
    sleep(3);
    float angle = 0;
    bool forward = 1;
    int ctr = 0;
    float time = 0;

    // Main loop
    while (ctr < MAX_CTR) {
        // Get feedback
        timespec start = time_s();

        if (ctr == 200)
            impulse = AVG_IMPULSE;
        else if (ctr == 600)
            impulse = 0;

        time = ctr * CTRL_PERIOD_US/1000000.0;

        if (ctr == 0)
            motorHandler.getPositions(fbckPositions, 0);
        else    
            motorHandler.getPositions(fbckPositions);

        // Save fbck position, impulse and time, Kp, Kd
        logStream << time << "," << impulse << "," << fbckPositions[0] << "," << Kp << "," << Kd << endl;    

        // Send new goal positions
        for (int i=0; i<nbrMotors; i++)
            goalPositions[i] = impulse;

        motorHandler.setPositions(goalPositions);


        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0) {
            toSleep_us = 0;
            //cout << "Overtime at step " << ctr << " , elapsed = " << elapsed << " us" << endl;
        }

        usleep(toSleep_us);
    }

    return(1);
}