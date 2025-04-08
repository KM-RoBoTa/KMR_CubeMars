/**
 ********************************************************************************************
 * @file    ex4_torque.cpp
 * @brief   Example for torque control
 * @details This is a very simple example to showcase torque control. \n
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

#define MAX_CTR         2000
#define CTRL_PERIOD_US  5000
#define GOAL_TORQUE1    +0.2
#define GOAL_TORQUE2    -0.2

using namespace std;

// --------------------------------------------------------------------------- //
//                                EDIT HERE 
// --------------------------------------------------------------------------- //

// Id(s) and model(s) of motor(s)
vector<int> ids = {1}; 
int nbrMotors = ids.size();
vector<KMR::CBM::Model> models{KMR::CBM::Model::AK60_6};

const char* can_bus = "can0";
// --------------------------------------------------------------------------- //


int main()
{
    KMR::CBM::MotorHandler motorHandler(ids, can_bus, models);
    
    // Create variables
    vector<float> goalPositions(nbrMotors, 0);
    vector<float> fbckPositions(nbrMotors), fbckSpeeds(nbrMotors), fbckTorques(nbrMotors);
    vector<int> fbckTemperatures(nbrMotors);
    sleep(1);

    // Set the 0 reference
    cout << "Setting 0" << endl;
    motorHandler.setZeroPosition();
    sleep(1);


    // ------------------ Main loop --------------------- //
    cout << endl << endl << " ---------- Torque control ---------" << endl;
    sleep(3);

    // Main loop
    vector<float> goalTorques(nbrMotors, 0);

    float torque = GOAL_TORQUE1; // Nm
    int ctr = 0, overtimeCtr = 0;

    // Start main loop
    while (ctr < MAX_CTR) {
        timespec start = KMR::CBM::time_s();

        // Get feedback
        if (ctr == 0)
            motorHandler.getTorques(fbckTorques, 0);
        else    
            motorHandler.getTorques(fbckTorques);

        cout << "Torques: "; 
        for (int i=0; i<nbrMotors; i++) {
            cout << fbckTorques[i] << " Nm";
            if (i != (nbrMotors-1))
                cout << ", ";
        }
        cout << endl;

        // Send new goal currents
        if (ctr > MAX_CTR/2)
            torque = GOAL_TORQUE2;
        for (int i=0; i<nbrMotors; i++)
            goalTorques[i] = torque;
        motorHandler.setTorques(goalTorques);

        // Increment counter and set the control loop to 5ms
        ctr++;

        timespec end = KMR::CBM::time_s();
        double elapsed = KMR::CBM::get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0) {
            toSleep_us = 0;
            overtimeCtr++;
            //cout << "Overtime at step " << ctr << " , elapsed = " << elapsed << " us" << endl;
        }
        usleep(toSleep_us);
    }

    cout << endl << endl << "The torque control example successfully finished." << endl;
    cout << endl;
    cout << "Loops longer than the control period due to scheduling delays: " << overtimeCtr <<
            "/" << ctr << " (" << (float)overtimeCtr/(float)ctr*100.0 << "%)" << endl; 

    motorHandler.getTemperatures(fbckTemperatures, 0);
    cout << endl;
    for (int i=0; i<nbrMotors; i++)
        cout << "Motor " << ids[i] << "'s temperature is " << fbckTemperatures[i] << " °C" << endl;

    return(1);
}