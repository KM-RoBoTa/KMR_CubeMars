/**
 ********************************************************************************************
 * @file    ex1_impedance.cpp
 * @brief   Example for impedance control
 * @details This is a very simple example to showcase impedance control. \n
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

#define INCREMENT       0.02
#define GOAL_SPEED1     +2*M_PI/3
#define GOAL_SPEED2     -2*M_PI/4

using namespace std;


// --------------------------------------------------------------------------- //
//                                EDIT HERE 
// --------------------------------------------------------------------------- //

// Id(s) and model(s) of motor(s)
vector<int> ids = {1}; 
int nbrMotors = ids.size();
vector<KMR::CBM::Model> models{KMR::CBM::Model::AK80_8};

const char* can_bus = "can0";
// --------------------------------------------------------------------------- //


int main()
{
    KMR::CBM::MotorHandler motorHandler(ids, can_bus, models);

    // Create variables
    float goalPosition = 0;
    float goalSpeed = 0;
    float goalTorque = 0.2;
    float Kp = 150;
    float Kd = 2;

    vector<float> positions(nbrMotors, 0);
    vector<float> speeds(nbrMotors, 0);
    vector<float> torques(nbrMotors, 0);
    vector<float> Kps(nbrMotors, 0);
    vector<float> Kds(nbrMotors,0);

    vector<float> fbckPositions(nbrMotors), fbckSpeeds(nbrMotors), fbckTorques(nbrMotors);
    vector<int> fbckTemperatures(nbrMotors);

    cout << "Setting 0" << endl;
    motorHandler.setZeroPosition(ids);
    sleep(1);

    // ------------------ Main loop --------------------- //
    cout << endl << endl << " ---------- Impedance control ---------" << endl;
    sleep(3);
    
    bool forward = 1;
    int ctr = 0, overtimeCtr = 0;

    while(ctr < MAX_CTR) {
        timespec start = KMR::CBM::time_s();

        // Get feedback
        if (ctr == 0)
            motorHandler.getFeedbacks(ids, fbckPositions, fbckSpeeds, fbckTorques, fbckTemperatures, 0);
        else    
            motorHandler.getFeedbacks(ids, fbckPositions, fbckSpeeds, fbckTorques, fbckTemperatures);

        cout << endl;
        for (int i=0; i<nbrMotors; i++) {
            cout << "Motor " << ids[i] << endl;
            cout << "\tPosition: " << fbckPositions[i] << " rad" << endl;
            cout << "\tSpeed: " << fbckSpeeds[i] << " rad/s" << endl;
            cout << "\tTorque: " << fbckTorques[i] << " Nm" << endl;
        }

        // Send new goal impedance
        motorHandler.setCommand(ids, positions, speeds, Kps, Kds, torques);

        // Update the goals for next loop
        if (forward) {
            goalPosition += INCREMENT;
            goalSpeed = GOAL_SPEED1;

            if (goalPosition > M_PI) {
                goalPosition = M_PI;
                forward = false;
            }
        }
        else {
            goalPosition -= INCREMENT;
            goalSpeed = GOAL_SPEED2;

            if (goalPosition < -M_PI) {
                goalPosition = -M_PI;
                forward = true;
            }
        }

        for (int i=0; i<nbrMotors; i++) {
            positions[i] = goalPosition;
            speeds[i] = goalSpeed;
            torques[i] = goalTorque;
            Kps[i] = Kp;
            Kds[i] = Kd;
        }

        // Increment counter and set the control loop to wanted frequency
        ctr++;

        timespec end = KMR::CBM::time_s();
        double elapsed = KMR::CBM::get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
            overtimeCtr++;
            //cout << "Overtime at step " << ctr << " , elapsed = " << elapsed << " us" << endl;
        usleep(toSleep_us);
    }

    cout << endl << endl << "The impedance control example successfully finished." << endl;
    cout << endl;
    cout << "Loops longer than the control period due to scheduling delays: " << overtimeCtr <<
            "/" << ctr << " (" << (float)overtimeCtr/(float)ctr*100.0 << "%)" << endl; 

    motorHandler.getTemperatures(fbckTemperatures, 0);
    cout << endl;
    for (int i=0; i<nbrMotors; i++)
        cout << "Motor " << ids[i] << "'s temperature is " << fbckTemperatures[i] << " °C" << endl;

    return(1);
}
