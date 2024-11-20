/**
 ********************************************************************************************
 * @file    ex5_gravity_compensation.cpp
 * @brief   Example for gravity compensation
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

#define g               9.81 // m/s²


using namespace std;

// Vars
vector<int> ids(1,1);
int nbrMotors = ids.size();
vector<KMR::CBM::Model> models(nbrMotors, KMR::CBM::Model::AK60_6);
const char* can_bus = "can0";
KMR::CBM::MotorHandler motorHandler(ids, can_bus, models);

// Fcts
void setZeroReference();
float getCompensationTorque(float goalAngle);

// =================================================
float weight = 0.100; // kg
float length = 0.010; // m
float goalPosition = -M_PI/2;
// =================================================


int main()
{
    setZeroReference();

    // Calculate compensation torque
    float compensationTorque = getCompensationTorque(goalPosition);

    // Set Kp and Kd
    vector<float> Kps(nbrMotors, 100);
    vector<float> Kds(nbrMotors, 2);
    motorHandler.setKps(ids, Kps);
    motorHandler.setKds(ids, Kds);

    // Main program 
    vector<float> goalPositions(nbrMotors, 0);
    vector<float> goalTorques(nbrMotors, 0);

    cout << endl << endl;
    cout << "Everything is now ready" << endl;
    cout << "First, the arm will execute the motion without torque compensation, then with it" << endl;
    cout << "When pressing y, you have 3 seconds to take some distance from the arm" << endl;
    cout << "Ready to continue? [y/n]" << endl;

    char c;
    while (c != 'y' && c != 'n') {
        cin >> c;

        if (c == 'y')
            break;
        else if (c == 'n') {
            cout << "Exiting" << endl;
            exit(1);
        }
    }
    cout << endl;
    cout << "Starting in 3" << endl;
    sleep(1);
    cout << "2" << endl;
    sleep(1);
    cout << "1" << endl;
    sleep(1);

    // Without compensation 
    motorHandler.setPositions(goalPositions);
    goalPositions[0] = goalPosition;
    sleep(3);
    motorHandler.setPositions(goalPositions);
    sleep(5);

    // With compensation 
    goalPositions[0] = 0;
    goalTorques[0] = 0;
    motorHandler.setPositions(goalPositions);
    sleep(3);
    goalPositions[0] = goalPosition;
    goalTorques[0] = compensationTorque;
    motorHandler.setPositions(goalPositions, goalTorques);
    sleep(5);

    // Ending program
    motorHandler.disableMotors();
    sleep(1);

    return(1);
}

void setZeroReference()
{
    cout << endl << endl;
    cout << "The 0 reference of the motor needs to be manually set" << endl;
    cout << "When pressing y, you have 3 seconds to adjust the motor to the wanted reference position" << endl;
    cout << "Ready to continue? [y/n]" << endl;

    char c;
    while (c != 'y' && c != 'n') {
        cin >> c;

        if (c == 'y')
            sleep(3);
        else if (c == 'n') {
            cout << "Exiting" << endl;
            exit(1);
        }
    }
    cout << endl;

    motorHandler.setZeroPosition();
    cout << "0 position set" << endl;
}

float getCompensationTorque(float goalAngle)
{
    // Calculate torque created by the bar 
    float tbar = -length * weight * g * sin(goalAngle);

    // Compensation torque = -tbar 
    return -tbar;
}