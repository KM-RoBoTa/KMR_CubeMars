/**
 ********************************************************************************************
 * @file    ex2_position.cpp
 * @brief   Example for position control
 * @details This is a very simple example to showcase position control. \n
 *          The motor's goal position oscillates between -pi and pi radians.
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

    // Set Kp and Kd
    vector<float> Kps(nbrMotors, 100);
    vector<float> Kds(nbrMotors, 2);
    motorHandler.setKps(ids, Kps);
    motorHandler.setKds(ids, Kds);
    
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
    cout << endl << endl << " ---------- Position control ---------" << endl;
    sleep(3);
    float angle = 0;
    bool forward = 1;
    int ctr = 0, overtimeCtr = 0;

    // Start main loop
    while (ctr < MAX_CTR) {
        timespec start = KMR::CBM::time_s();

        // Get feedback
        if (ctr == 0)
            motorHandler.getPositions(fbckPositions, 0);
        else    
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
            angle -= INCREMENT;

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
            overtimeCtr++;
            //cout << "Overtime at step " << ctr << " , elapsed = " << elapsed << " us" << endl;
        }

        usleep(toSleep_us);
    }

    cout << endl << endl << "The position control example successfully finished." << endl;
    cout << endl;
    cout << "Loops longer than the control period due to scheduling delays: " << overtimeCtr <<
            "/" << ctr << " (" << (float)overtimeCtr/(float)ctr*100.0 << "%)" << endl; 

    motorHandler.getTemperatures(fbckTemperatures, 0);
    cout << endl;
    for (int i=0; i<nbrMotors; i++)
        cout << "Motor " << ids[i] << "'s temperature is " << fbckTemperatures[i] << " °C" << endl;

    return(1);
}
