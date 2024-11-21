/**
 ********************************************************************************************
 * @file    ex2_position.cpp
 * @brief   Example for position control
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

#define INCREMENT       0.02
#define MAX_CTR         1000
#define CTRL_PERIOD_US  5000

using namespace std;

int main()
{
    vector<int> ids(1,1);
    int nbrMotors = ids.size();
    vector<KMR::CBM::Model> models(nbrMotors, KMR::CBM::Model::AK60_6);

    const char* can_bus = "can0";
    KMR::CBM::MotorHandler motorHandler(ids, can_bus, models);

    // Set Kp and Kd
    vector<float> Kps(nbrMotors, 100);
    vector<float> Kds(nbrMotors, 2);
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
    cout << "Sending the command" << endl;

    cout << endl << endl << " ---------- Position control ---------" << endl;
    sleep(3);
    float angle = 0;
    bool forward = 1;
    int ctr = 0;

    // Main loop
    while (ctr < MAX_CTR) {
        // Get feedback
        timespec start = time_s();

        motorHandler.getPositions(fbckPositions);

        cout << "Positions: "; 
        for (int i=0; i<nbrMotors; i++) {
            cout << fbckPositions[i] << " rad";
            if (i != (nbrMotors-1))
                cout << ", ";
        }
        cout << endl;

        // Send new goal positions
        for (int i=0; i<nbrMotors; i++)
            goalPositions[i] = angle;
        //timespec startCommand = time_s();
        motorHandler.setPositions(goalPositions);
        //timespec endCommand = time_s();
        //double elapsedCommand = get_delta_us(endCommand, startCommand);
        //cout << "Elapsed = " << elapsedCommand/1000.0 << " ms" << endl;

        // Update the goal angle for next loop
        if (forward) {
            angle += INCREMENT;

            if (angle > M_PI) {
                angle = M_PI;
                forward = false;
            }
        }
        else {
            angle -= INCREMENT;

            if (angle < -M_PI) {
                angle = -M_PI;
                forward = true;
            }
        }

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = time_s();
        double elapsed = get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0) {
            toSleep_us = 0;
            cout << "Overtime at step " << ctr << " , elapsed = " << elapsed << " us" << endl;
        }

        usleep(toSleep_us);
    }

    return(1);
}