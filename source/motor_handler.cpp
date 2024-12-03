/**
 ******************************************************************************
 * @file            motor_handler.cpp
 * @brief           Methods of the MotorHandler class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 11/2024
 *****************************************************************************
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

const int SOCKET_TIMEOUT_US = 30*1000; // 30ms in us

using namespace std;

namespace KMR::CBM
{

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

    pingMotors();
}

MotorHandler::~MotorHandler()
{
    bool success = disableMotors();
    if (!success) {
        cout << "Error! Motors were not disabled correctly. ";
        cout << "Disconnect power after the program ends" << endl;
    }

    delete m_listener;
    delete m_writer;

    for (int i=0; i<m_nbrMotors; i++)
        delete m_motors[i];

    // close socket
}

int MotorHandler::openSocket(const char* can_bus)
{
    // Create a CAN socket
    //int s = socket(PF_CAN, SOCK_RAW|SOCK_NONBLOCK, CAN_RAW);
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        cout << "Error creating the socket! Exiting" << endl;
        exit(1);
    }
    else   
        cout << "Socket created successfully" << endl;

    // Set socket timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = SOCKET_TIMEOUT_US;
    int success = setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
    if (success < 0) {
        cout << "Error setting the timeout to the CAN socket" << endl;
        exit(1);
    }

    // Bind the socket to the CAN bus
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
        cout << "Socket successfully bound to the CAN bus " << endl;

    return s;
}

void MotorHandler::pingMotors()
{
    for(auto id : m_ids) {
        if(m_writer->writeEnterMITMode(id) < 0)
            cout << "[FAILED REQUEST] Failed to ping motor " << id << endl;
    }

    for(auto id : m_ids) {
        bool success = m_listener->fbckReceived(id);
        if (success)
            cout << "Motor " << id << " pinged successfully!" << endl;
        else {
            cout << "Error! Motor " << id << " is not responding" << endl;
            exit(1);
        }
    }
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

bool MotorHandler::enableMotors(vector<int> ids)
{
    for(auto id : ids) {
        if(m_writer->writeEnterMITMode(id) < 0)
            cout << "[FAILED REQUEST] Failed to send the enable command to motor " << id << endl;
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

bool MotorHandler::enableMotors()
{
    return(enableMotors(m_ids));
}

bool MotorHandler::disableMotors(vector<int> ids)
{
    // Stop the motors first
    bool success = stopMotors(ids);
    if (!success)
        cout << "Error! The motors were not stopped correctly. Power them down after end of program" << endl;

    for(auto id : ids) {
        if(m_writer->writeExitMITMode(id) < 0)
            cout << "[FAILED REQUEST] Failed to send the disabling command to motor " << id << endl;
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


bool MotorHandler::disableMotors()
{
    return(disableMotors(m_ids));
}


bool MotorHandler::setZeroPosition(vector<int> ids)
{
    for(auto id : ids) {
        if(m_writer->writeZeroPosition(id) < 0)
            cout << "[FAILED REQUEST] Failed to send zero-position to motor " << id << endl;
    }

    int fullSuccess = 0;
    for(auto id : ids) {
        bool success = m_listener->fbckReceived(id);
        fullSuccess += success;
    }

    // Maintain the position 
    bool success2 = maintainPosition(ids, 0);

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
bool MotorHandler::setImpedance(vector<int> ids, vector<float> positions, vector<float> speeds,
                                   vector<float> Kps, vector<float> Kds, vector<float> torques)
{
    for (int i=0; i<ids.size(); i++) {
        if(m_writer->writeMITCommand(ids[i], positions[i], speeds[i], Kps[i], Kds[i], torques[i]) < 0)
            cout << "[FAILED REQUEST] Failed to send impendance command to motor " << ids[i] << endl;
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

bool MotorHandler::setImpedance(vector<float> positions, vector<float> speeds,
                                   vector<float> Kps, vector<float> Kds, vector<float> torques)
{
    return(setImpedance(m_ids, positions, speeds, Kps, Kds, torques));
}


bool MotorHandler::setPositions(vector<int> ids, vector<float> positions)
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
    bool success = setImpedance(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

bool MotorHandler::setPositions(vector<float> positions)
{
    return (setPositions(m_ids, positions));
}

bool MotorHandler::setPositions(vector<int> ids, vector<float> positions, vector<float> torques)
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
    bool success = setImpedance(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

bool MotorHandler::setPositions(vector<float> positions, vector<float> torques)
{
   return (setPositions(m_ids, positions, torques));
}


bool MotorHandler::setSpeeds(vector<int> ids, vector<float> speeds)
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
    bool success = setImpedance(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

bool MotorHandler::setSpeeds(vector<float> speeds)
{
    return (setSpeeds(m_ids, speeds));
}

bool MotorHandler::setSpeeds(vector<int> ids, vector<float> speeds, vector<float> torques)
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
    bool success = setImpedance(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

bool MotorHandler::setSpeeds(vector<float> speeds, vector<float> torques)
{
   return (setSpeeds(m_ids, speeds, torques));
}


bool MotorHandler::setTorques(vector<int> ids, vector<float> torques)
{
    vector<float> Kps(ids.size(), 0);
    vector<float> Kds(ids.size(), 0);
    vector<float> positions(ids.size(), 0);
    vector<float> speeds(ids.size(), 0);

    // Send the command
    bool success = setImpedance(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

bool MotorHandler::setTorques(vector<float> torques)
{
    return (setTorques(m_ids, torques));
}


bool MotorHandler::stopMotors(vector<int> ids)
{
    int nbrMotors = ids.size();
    vector<float> positions(nbrMotors, 0);
    vector<float> speeds(nbrMotors, 0);
    vector<float> torques(nbrMotors, 0);
    vector<float> Kps(nbrMotors, 0);
    vector<float> Kds(nbrMotors, 0);

    bool success = setImpedance(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

bool MotorHandler::stopMotors()
{
    return(stopMotors(m_ids));
}

bool MotorHandler::stopMotor(int id)
{
    vector<int> ids(1, id);
    return(stopMotors(ids));
}

bool MotorHandler::maintainPosition(vector<int> ids, bool moving)
{
    vector<float> fbckPositions(ids.size(), 0);
    bool success = getPositions(ids, fbckPositions, moving);
    if (!success) {
        cout << "Error! Could not get fbck positions to maintain position" << endl;
        return 0;
    }

    success = setPositions(ids, fbckPositions);
    return success;
}

bool MotorHandler::maintainPosition(bool moving)
{
    return(maintainPosition(m_ids, moving));
}


/*
 *****************************************************************************
 *                           Getting feedback
 ****************************************************************************/

bool MotorHandler::getFeedbacks(vector<int> ids, vector<float>& fbckPositions, vector<float>& fbckSpeeds,
                                vector<float>& fbckTorques, vector<int>& fbckTemperatures, bool moving)
{
    for(auto id : ids) {
        if (!moving) {
            if(m_writer->writeEnterMITMode(id) < 0)  // Entering mode is also the command for fbck request
                cout << "[FAILED REQUEST] Failed to request feedback for motor " << id << endl;
        }
        else {
            if(m_writer->writePreviousCommand(id) < 0)  // When moving, resend previous command
                cout << "[FAILED REQUEST] Failed to request feedback for motor " << id << endl;
        }
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
                                vector<float>& fbckTorques, vector<int>& fbckTemperatures, bool moving)
{
    return(getFeedbacks(m_ids, fbckPositions, fbckSpeeds, fbckTorques, fbckTemperatures, moving));
}


bool MotorHandler::getPositions(vector<int> ids, vector<float>& fbckPositions, bool moving)
{
    for(auto id : ids) {
        if (!moving) {
            if(m_writer->writeEnterMITMode(id) < 0)  // Entering mode is also the command for fbck request
                cout << "[FAILED REQUEST] Failed to request position feedback for motor " << id << endl;
        }
        else {
            if(m_writer->writePreviousCommand(id) < 0)  // When moving, resend previous command
                cout << "[FAILED REQUEST] Failed to request position feedback for motor " << id << endl;
        }
    }

    int nbrMotors = ids.size();
    vector<float> fbckSpeeds(nbrMotors, 0);
    vector<float> fbckTorques(nbrMotors, 0);
    vector<int> fbckTemperatures(nbrMotors, 0);

    int fullSuccess = 0;
    for (int i=0; i<nbrMotors; i++) {
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

bool MotorHandler::getPositions(vector<float>& fbckPositions, bool moving)
{
    return(getPositions(m_ids, fbckPositions, moving));
}


bool MotorHandler::getSpeeds(vector<int> ids, vector<float>& fbckSpeeds, bool moving)
{
    for(auto id : ids) {
        if (!moving) {
            if(m_writer->writeEnterMITMode(id) < 0)  // Entering mode is also the command for fbck request
                cout << "[FAILED REQUEST] Failed to request speed feedback for motor " << id << endl;
        }
        else {
            if(m_writer->writePreviousCommand(id) < 0)  // When moving, resend previous command
                cout << "[FAILED REQUEST] Failed to request speed feedback for motor " << id << endl;
        }
    }

    int nbrMotors = ids.size();
    vector<float> fbckPositions(nbrMotors, 0);
    vector<float> fbckTorques(nbrMotors, 0);
    vector<int> fbckTemperatures(nbrMotors, 0);

    int fullSuccess = 0;
    for (int i=0; i<nbrMotors; i++) {
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

bool MotorHandler::getSpeeds(vector<float>& fbckSpeeds, bool moving)
{
    return(getSpeeds(m_ids, fbckSpeeds, moving));
}

bool MotorHandler::getTorques(vector<int> ids, vector<float>& fbckTorques, bool moving)
{
    for(auto id : ids) {
        if (!moving) {
            if(m_writer->writeEnterMITMode(id) < 0)  // Entering mode is also the command for fbck request
                cout << "[FAILED REQUEST] Failed to request torque feedback for motor " << id << endl;
        }
        else {
            if(m_writer->writePreviousCommand(id) < 0)  // When moving, resend previous command
                cout << "[FAILED REQUEST] Failed to request torque feedback for motor " << id << endl;
        }
    }

    int nbrMotors = ids.size();
    vector<float> fbckPositions(nbrMotors, 0);
    vector<float> fbckSpeeds(nbrMotors, 0);
    vector<int> fbckTemperatures(nbrMotors, 0);

    int fullSuccess = 0;
    for (int i=0; i<nbrMotors; i++) {
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

bool MotorHandler::getTorques(vector<float>& fbckTorques, bool moving)
{
    return(getTorques(m_ids, fbckTorques, moving));
}

bool MotorHandler::getTemperatures(vector<int> ids, vector<int>& fbckTemperatures, bool moving)
{
    for(auto id : ids) {
        if (!moving) {
            if(m_writer->writeEnterMITMode(id) < 0)  // Entering mode is also the command for fbck request
                cout << "[FAILED REQUEST] Failed to request temperature feedback for motor " << id << endl;
        }
        else {
            if(m_writer->writePreviousCommand(id) < 0)  // When moving, resend previous command
                cout << "[FAILED REQUEST] Failed to request temperature feedback for motor " << id << endl;
        }
    }

    int nbrMotors = ids.size();
    vector<float> fbckPositions(nbrMotors, 0);
    vector<float> fbckSpeeds(nbrMotors, 0);
    vector<float> fbckTorques(nbrMotors, 0);

    int fullSuccess = 0;
    for (int i=0; i<nbrMotors; i++) {
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

bool MotorHandler::getTemperatures(vector<int>& fbckTemperatures, bool moving)
{
    return(getTemperatures(m_ids, fbckTemperatures, moving));
}


}