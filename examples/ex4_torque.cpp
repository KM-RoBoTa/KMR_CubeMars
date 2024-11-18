/**
 ********************************************************************************************
 * @file    ex4_torque.cpp
 * @brief   Example for torque control
 * @details 
 ********************************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 ********************************************************************************************
 */

#include "motor_handler.hpp"
#include "unistd.h"
#include <cmath>

#define MAX_CTR         2000
#define CTRL_PERIOD_US  5000
#define GOAL_TORQUE1    +0.1
#define GOAL_TORQUE2    -0.1

using namespace std;

int main()
{
    vector<int> ids(1,1);
    int nbrMotors = ids.size();
    vector<KMR::CBM::Model> models(nbrMotors, KMR::CBM::Model::AK60_6);

    const char* can_bus = "can0";
    KMR::CBM::MotorHandler motorHandler(ids, can_bus, models);
    
    // Enable motor
    vector<float> goalPositions(nbrMotors, 0);
    vector<float> fbckPositions(nbrMotors), fbckSpeeds(nbrMotors), fbckTorques(nbrMotors);
    vector<int> fbckTemperatures(nbrMotors);
    cout << "Going to MIT mode" << endl;
    motorHandler.enableMotors();
    sleep(1);

    // Set the 0 reference
    /*cout << "Setting 0" << endl;
    motorHandler.setZeroPosition();
    sleep(1);*/

    // Main loop
    cout << "Sending the command" << endl;

    cout << endl << endl << " ---------- Torque control ---------" << endl;
    sleep(3);

    // Main loop
    vector<float> goalTorques(nbrMotors, 0);

    float torque = GOAL_TORQUE1; // Nm
    int ctr = 0;

    while (ctr < MAX_CTR) {
        // Get feedback
        timespec start = time_s();
        /*robot.getCurrents(fbckCurrents);

        cout << "Currents: "; 
        for (int i=0; i<nbrMotors; i++) {
            cout << fbckCurrents[i] << " A";
            if (i != (nbrMotors-1))
                cout << ", ";
        }
        cout << endl;*/

        // Send new goal currents
        if (ctr > MAX_CTR/2)
            torque = GOAL_TORQUE2;
        for (int i=0; i<nbrMotors; i++)
            goalTorques[i] = torque;
        motorHandler.setTorques(goalTorques);

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
        usleep(toSleep_us);
    }

    cout << "Exiting MIT mode" << endl;
    motorHandler.disableMotors();
    sleep(1);

    return(1);
}