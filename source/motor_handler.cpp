/**
 ******************************************************************************
 * @file        motor_handler.cpp
 * @brief       Handles the communication with MyActuator Motors
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ******************************************************************************
 */

// Standard libraries
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <cmath>

#include <sys/socket.h>
#include <linux/can.h>
#include <sys/ioctl.h>
#include <cstring>
#include <net/if.h> // network devices
#include <sstream> // Convert dec-hex

#include "motor_handler.hpp"

// TODO: signs for speed & torque?

using namespace std;

MotorHandler::MotorHandler(vector<int> ids, const char* can_bus, vector<Model> models)
{
    if (models.size() != ids.size()) {
        cout << "Error! The size of the models' vector is not equal to the size of motor ids" << endl;
        exit(1);
    }

    m_ids = ids;
    m_nbrMotors = m_ids.size();

    for (int i=0; i<ids.size(); i++) {
        Motor* motor = new Motor(ids[i], models[i]);
        m_motors.push_back(motor);
    }

    // Open a socket to be able to communicate over a CAN network
    int s = openSocket(can_bus);

    // Create the writer and the listener 
    m_writer = new Writer(m_motors, ids, s);
    m_listener = new Listener(m_motors, ids, s);

    usleep(2*1000000);
}

MotorHandler::~MotorHandler()
{
    delete m_listener;
    delete m_writer;

    for (int i=0; i<m_nbrMotors; i++)
        delete m_motors[i];

    // close socket
}

int MotorHandler::openSocket(const char* can_bus)
{
    int s = socket(PF_CAN, SOCK_RAW|SOCK_NONBLOCK, CAN_RAW);
    if (s < 0) {
        cout << "Error opening the socket! Exiting" << endl;
        exit(1);
    }
    else   
        cout << "Socket opened successfully" << endl;

    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_bus);
    ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));  // Init
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int result = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (result < 0) {
        cout << "Binding error! Exiting" << endl;
        exit(1);
    }
    else
        cout << "Binding ok " << endl;

    return s;
}


void MotorHandler::setKps(vector<int> ids, vector<float> Kps)
{
    for(auto id : ids) {
        int idx = getIndex(m_ids, id);
        m_motors[idx]->Kp = Kps[idx];
    }
}

void MotorHandler::setKds(vector<int> ids, vector<float> Kds)
{
    for(auto id : ids) {
        int idx = getIndex(m_ids, id);
        m_motors[idx]->Kd = Kds[idx];
    }
}


/*
 *****************************************************************************
 *                               Motor infos
 ****************************************************************************/

bool MotorHandler::enterMITMode(vector<int> ids)
{
    for(auto id : ids) {
        if(m_writer->writeEnterMITMode(id) < 0)
            cout << "[FAILED REQUEST] Failed to send PID-reading for motor " << id << endl;
    }

    int fullSuccess = 0;
    for(auto id : ids) {
        bool success = m_listener->fbckReceived(id);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::enterMITMode()
{
    return(enterMITMode(m_ids));
}

bool MotorHandler::exitMITMode(vector<int> ids)
{
    for(auto id : ids) {
        if(m_writer->writeExitMITMode(id) < 0)
            cout << "[FAILED REQUEST] Failed to send PID-reading for motor " << id << endl;
    }

    int fullSuccess = 0;
    for(auto id : ids) {
        bool success = m_listener->fbckReceived(id);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0; 
}


bool MotorHandler::exitMITMode()
{
    return(exitMITMode(m_ids));
}


bool MotorHandler::setZeroPosition(vector<int> ids)
{
    for(auto id : ids) {
        if(m_writer->writeZeroPosition(id) < 0)
            cout << "[FAILED REQUEST] Failed to send zero-position for motor " << id << endl;
    }

    int fullSuccess = 0;
    for(auto id : ids) {
        bool success = m_listener->fbckReceived(id);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::setZeroPosition()
{
    return(setZeroPosition(m_ids));
}

bool MotorHandler::setZeroPosition(int id)
{
    vector<int> ids(1, id);
    return(setZeroPosition(ids));
}


// Arguments: desired positions & speeds
bool MotorHandler::sendImpedanceCommand(vector<int> ids, vector<float> positions, vector<float> speeds,
                                   vector<float> Kps, vector<float> Kds, vector<float> torques)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writeMITCommand(ids[i], positions[i], speeds[i], Kps[i], Kds[i], torques[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send MIT command to motor " << ids[i] << endl;
    }

    int fullSuccess = 0;
    for(auto id : ids) {
        bool success = m_listener->fbckReceived(id);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0; 
}

bool MotorHandler::sendImpedanceCommand(vector<float> positions, vector<float> speeds,
                                   vector<float> Kps, vector<float> Kds, vector<float> torques)
{
    return(sendImpedanceCommand(m_ids, positions, speeds, Kps, Kds, torques));
}


bool MotorHandler::getFeedbacks(vector<int> ids, vector<float>& fbckPositions, vector<float>& fbckSpeeds,
                                vector<float>& fbckTorques, vector<int>& fbckTemperatures)
{
    for(auto id : ids) {
        if(m_writer->writeEnterMITMode(id) < 0)  // Entering mode is also the command for fbck request
            cout << "[FAILED REQUEST] Failed to request feedback for motor " << id << endl;
    }

    int fullSuccess = 0;
    for (int i=0; i<ids.size(); i++) {
        float temp = 0;
        bool success = m_listener->getFeedbacks(ids[i], fbckPositions[i], fbckSpeeds[i],
                                                fbckTorques[i], fbckTemperatures[i]);
        fullSuccess += success;
    }

    // If no timeout for any motor, return 1. Else, return 0
    if (fullSuccess == ids.size())
        return 1;
    else
        return 0;  
}

bool MotorHandler::getFeedbacks(vector<float>& fbckPositions, vector<float>& fbckSpeeds,
                                vector<float>& fbckTorques, vector<int>& fbckTemperatures)
{
    return(getFeedbacks(m_ids, fbckPositions, fbckSpeeds, fbckTorques, fbckTemperatures));
}

bool MotorHandler::sendPosition(vector<int> ids, vector<float> positions)
{
    vector<float> Kps(ids.size());
    vector<float> Kds(ids.size());
    vector<float> speeds(ids.size(), 0);
    vector<float> torques(ids.size(), 0);

    // Fetch the previously set Kp and Kd for each motor
    for(auto id : ids) {
        int idx = getIndex(m_ids, id);
        float Kp = m_motors[idx]->Kp;
        if (Kp < 0) {
            cout << "Error! Kp not set for motor " << id << endl;
            exit(1);
        }
        else 
            Kps[idx] = Kp;

        float Kd = m_motors[idx]->Kd;
        if (Kd < 0) {
            cout << "Error! Kd not set for motor " << id << endl;
            exit(1);
        }
        else 
            Kds[idx] = Kd;
    }

    // Send the command
    bool success = sendImpedanceCommand(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

bool MotorHandler::sendPosition(vector<float> positions)
{
    return (sendPosition(m_ids, positions));
}

bool MotorHandler::sendPosition(vector<int> ids, vector<float> positions, vector<float> torques)
{
    vector<float> Kps(ids.size());
    vector<float> Kds(ids.size());
    vector<float> speeds(ids.size(), 0);

    // Fetch the previously set Kp and Kd for each motor
    for(auto id : ids) {
        int idx = getIndex(m_ids, id);
        float Kp = m_motors[idx]->Kp;
        if (Kp < 0) {
            cout << "Error! Kp not set for motor " << id << endl;
            exit(1);
        }
        else 
            Kps[idx] = Kp;

        float Kd = m_motors[idx]->Kd;
        if (Kd < 0) {
            cout << "Error! Kd not set for motor " << id << endl;
            exit(1);
        }
        else 
            Kds[idx] = Kd;
    }

    // Send the command
    bool success = sendImpedanceCommand(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

bool MotorHandler::sendPosition(vector<float> positions, vector<float> torques)
{
   return (sendPosition(m_ids, positions, torques));
}


bool MotorHandler::sendSpeed(vector<int> ids, vector<float> speeds)
{
    vector<float> Kps(ids.size(), 0);
    vector<float> Kds(ids.size());
    vector<float> positions(ids.size(), 0);
    vector<float> torques(ids.size(), 0);

    // Fetch the previously set Kd for each motor
    for(auto id : ids) {
        int idx = getIndex(m_ids, id);

        float Kd = m_motors[idx]->Kd;
        if (Kd < 0) {
            cout << "Error! Kd not set for motor " << id << endl;
            exit(1);
        }
        else 
            Kds[idx] = Kd;
    }

    // Send the command
    bool success = sendImpedanceCommand(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

bool MotorHandler::sendSpeed(vector<float> speeds)
{
    return (sendSpeed(m_ids, speeds));
}

bool MotorHandler::sendSpeed(vector<int> ids, vector<float> speeds, vector<float> torques)
{
    vector<float> Kps(ids.size(), 0);
    vector<float> Kds(ids.size());
    vector<float> positions(ids.size(), 0);

    // Fetch the previously set Kp and Kd for each motor
    for(auto id : ids) {
        int idx = getIndex(m_ids, id);

        float Kd = m_motors[idx]->Kd;
        if (Kd < 0) {
            cout << "Error! Kd not set for motor " << id << endl;
            exit(1);
        }
        else 
            Kds[idx] = Kd;
    }

    // Send the command
    bool success = sendImpedanceCommand(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

bool MotorHandler::sendSpeed(vector<float> speeds, vector<float> torques)
{
   return (sendSpeed(m_ids, speeds, torques));
}


bool MotorHandler::sendTorque(vector<int> ids, vector<float> torques)
{
    vector<float> Kps(ids.size(), 0);
    vector<float> Kds(ids.size(), 0);
    vector<float> positions(ids.size(), 0);
    vector<float> speeds(ids.size(), 0);

    // Send the command
    bool success = sendImpedanceCommand(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

bool MotorHandler::sendTorque(vector<float> torques)
{
    return (sendSpeed(m_ids, torques));
}