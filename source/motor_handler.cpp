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

/**
 *  @brief 		Constructor for MotorHandler
 * 	@param[in] 	ids IDs of all motors 
 *  @param[in]  can_bus CAN bus name used for the communication with the motors
 *  @param[in]  models CubeMars models of all motors
 */ 
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
    m_socket = openSocket(can_bus);

    // Create the writer and the listener 
    m_writer = new Writer(m_motors, ids, m_socket);
    m_listener = new Listener(m_motors, ids, m_socket);

    usleep(2*1000000);

    pingMotors();
}

/**
 *  @brief  Destructor for MotorHandler
 */ 
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

    // Close socket
    close(m_socket);
}

/**
 *  @brief      Open the CAN communication socket
 *  @param[in]  can_bus CAN bus name used for the communication with the motors 
 *  @retval     Socket's file descriptor
 */ 
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

/**
 *  @brief  Ping motors to check the communication is working properly
 */ 
void MotorHandler::pingMotors()
{
    for(auto id : m_ids) {
        if(m_writer->writeEnterMITMode(id) < 0)
            cout << "[FAILED REQUEST] Failed to ping motor " << id << endl;
    }

    cout << endl;
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

/**
 *  @brief      Set Kp to each motor
 *  @param[in]  ids Motor IDs for which we want to save Kp
 *  @param[in]  Kps List of Kps saved for the list of motors
 */ 
void MotorHandler::setKps(vector<int> ids, vector<float> Kps)
{
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        m_motors[idx]->Kp = Kps[i];
    }
}

/**
 *  @brief      Set Kd to each motor
 *  @param[in]  ids Motor IDs for which we want to save Kd
 *  @param[in]  Kds List of Kds saved for the list of motors
 */ 
void MotorHandler::setKds(vector<int> ids, vector<float> Kds)
{
    for (int i=0; i<ids.size(); i++) {
        int idx = getIndex(m_ids, ids[i]);
        m_motors[idx]->Kd = Kds[i];
    }
}


/*
 *****************************************************************************
 *                         Enable/disable + zero-setting
 ****************************************************************************/

/**
 *  @brief      Enable the input motors: enter MIT mode
 *  @param[in]  ids Motor IDs to be enabled
 *  @retval     1 if motors successfully enabled, 0 otherwise
 */ 
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

/**
 *  @brief      Enable all motors: enter MIT modes
 *  @retval     1 if motors successfully enabled, 0 otherwise
 */ 
bool MotorHandler::enableMotors()
{
    return(enableMotors(m_ids));
}

/**
 *  @brief      Disable the input motors: exit MIT mode
 *  @param[in]  ids Motor IDs to be disabled
 *  @retval     1 if motors successfully disabled, 0 otherwise
 */ 
bool MotorHandler::disableMotors(vector<int> ids)
{
    // Stop the motors first ("parking" mode)
    bool success = stopMotors(ids);
    if (!success)
        cout << "Error! The motors were not stopped correctly. Power them down after end of program" << endl;

    // Exit the MIT mode
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

/**
 *  @brief      Disable all motors: exit MIT mode
 *  @retval     1 if motors successfully disabled, 0 otherwise
 */ 
bool MotorHandler::disableMotors()
{
    return(disableMotors(m_ids));
}

/**
 *  @brief      Stop input motors: torque off
 *  @note       Corresponds to the "parking mode" in CubeMars's videos
 *  @param[in]  ids IDs of the motors to stop
 *  @retval     1 if motors successfully stopped, 0 otherwise
 */ 
bool MotorHandler::stopMotors(vector<int> ids)
{
    int nbrMotors = ids.size();
    vector<float> positions(nbrMotors, 0);
    vector<float> speeds(nbrMotors, 0);
    vector<float> torques(nbrMotors, 0);
    vector<float> Kps(nbrMotors, 0);
    vector<float> Kds(nbrMotors, 0);

    bool success = setCommand(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

/**
 *  @brief      Stop all motors: torque off
 *  @note       Corresponds to the "parking mode" in CubeMars's videos
 *  @retval     1 if motors successfully stopped, 0 otherwise
 */ 
bool MotorHandler::stopMotors()
{
    return(stopMotors(m_ids));
}

/**
 *  @brief      Stop a motor: torque off
 *  @note       Corresponds to the "parking mode" in CubeMars's videos
 *  @retval     1 if motor successfully stopped, 0 otherwise
 */ 
bool MotorHandler::stopMotor(int id)
{
    vector<int> ids(1, id);
    return(stopMotors(ids));
}

/**
 *  @brief      Set the zero position of input motors at their current position
 *  @details    After setting the zero, it appeared necessary to send the maintainPosition command,
 *              otherwise the motors would often execute the previous control command a second time
 *  @param[in]  ids Motor IDs setting their zero reference
 *  @retval     1 if motors successfully set their zero, 0 otherwise
 */ 
bool MotorHandler::setZeroPosition(vector<int> ids)
{
    // Stop the motors before setting their 0-position ("parking mode")
    bool motorsStopped = stopMotors(ids);
    if (!motorsStopped) {
        cout << "[FAILED REQUEST] Failed to stop all motors before setting their zero" << endl;
        return 0; 
    }

    // Send the 0-setting command
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

/**
 *  @brief      Set the zero position of all motors at their current position
 *  @details    After setting the zero, it appeared necessary to send the maintainPosition command,
 *              otherwise the motors would often execute the previous control command a second time
 *  @retval     1 if motors successfully set their zero, 0 otherwise
 */ 
bool MotorHandler::setZeroPosition()
{
    return(setZeroPosition(m_ids));
}

/**
 *  @brief      Set the zero position of the input motor at its current position
 *  @details    After setting the zero, it appeared necessary to send the maintainPosition command,
 *              otherwise the motors would often execute the previous control command a second time
 *  @param[in]  id Motor ID setting its zero reference
 *  @retval     1 if the motor successfully set its zero, 0 otherwise
 */ 
bool MotorHandler::setZeroPosition(int id)
{
    vector<int> ids(1, id);
    return(setZeroPosition(ids));
}

/*
 *****************************************************************************
 *                           Control commands
 ****************************************************************************/

/**
 * @brief       Send MIT control commands to the listed motors
 * @param[in]   ids IDs of the motors receiving the commands
 * @param[in]   positions Goal positions [rad]
 * @param[in]   speeds Goal speeds [rad/s]
 * @param[in]   Kps Kp factors
 * @param[in]   Kds Kd factors
 * @param[in]   torques Feedforward torques [Nm]
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::setCommand(vector<int> ids, vector<float> positions, vector<float> speeds,
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


/**
 * @brief       Send MIT control commands to all motors
 * @param[in]   positions Goal positions [rad]
 * @param[in]   speeds Goal speeds [rad/s]
 * @param[in]   Kps Kp factors
 * @param[in]   Kds Kd factors
 * @param[in]   torques Feedforward torques [Nm]
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::setCommand(vector<float> positions, vector<float> speeds,
                                   vector<float> Kps, vector<float> Kds, vector<float> torques)
{
    return(setCommand(m_ids, positions, speeds, Kps, Kds, torques));
}


/**
 * @brief       Send position control commands to input motors
 * @param[in]   ids IDs of the motors receiving the commands
 * @param[in]   positions Goal positions [rad]
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::setPositions(vector<int> ids, vector<float> positions)
{
    vector<float> Kps(ids.size(), 0);
    vector<float> Kds(ids.size(), 0);
    vector<float> speeds(ids.size(), 0);
    vector<float> torques(ids.size(), 0);

    // Fetch the previously set Kp and Kd for each motor
    for (int i=0; i<ids.size(); i++) {
        int id = ids[i];
        int idx = getIndex(m_ids, id);
        float Kp = m_motors[idx]->Kp;
        if (Kp < 0) {
            cout << "Error! Kp not set for motor " << id << endl;
            exit(1);
        }
        else 
            Kps[i] = Kp;

        float Kd = m_motors[idx]->Kd;
        if (Kd < 0) {
            cout << "Error! Kd not set for motor " << id << endl;
            exit(1);
        }
        else 
            Kds[i] = Kd;
    }

    // Send the command
    bool success = setCommand(ids, positions, speeds, Kps, Kds, torques);
    return success;
}


/**
 * @brief       Send position control commands to all motors
 * @param[in]   positions Goal positions [rad]
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::setPositions(vector<float> positions)
{
    return (setPositions(m_ids, positions));
}

/**
 * @brief       Send position control commands with FF torques to input motors
 * @param[in]   ids IDs of the motors receiving the commands
 * @param[in]   positions Goal positions [rad]
 * @param[in]   torques Feedforward torques [Nm]
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::setPositions(vector<int> ids, vector<float> positions, vector<float> torques)
{
    vector<float> Kps(ids.size(), 0);
    vector<float> Kds(ids.size(), 0);
    vector<float> speeds(ids.size(), 0);

    // Fetch the previously set Kp and Kd for each motor
    for (int i=0; i<ids.size(); i++) {
        int id = ids[i];
        int idx = getIndex(m_ids, id);
        float Kp = m_motors[idx]->Kp;
        if (Kp < 0) {
            cout << "Error! Kp not set for motor " << id << endl;
            exit(1);
        }
        else 
            Kps[i] = Kp;

        float Kd = m_motors[idx]->Kd;
        if (Kd < 0) {
            cout << "Error! Kd not set for motor " << id << endl;
            exit(1);
        }
        else 
            Kds[i] = Kd;
    }

    // Send the command
    bool success = setCommand(ids, positions, speeds, Kps, Kds, torques);
    return success;
}


/**
 * @brief       Send position control commands with FF torques to all motors
 * @param[in]   positions Goal positions [rad]
 * @param[in]   torques Feedforward torques [Nm]
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::setPositions(vector<float> positions, vector<float> torques)
{
   return (setPositions(m_ids, positions, torques));
}


/**
 * @brief       Send speed control commands to input motors
 * @param[in]   ids IDs of the motors receiving the commands
 * @param[in]   speeds Goal speeds [rad/s]
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::setSpeeds(vector<int> ids, vector<float> speeds)
{
    vector<float> Kps(ids.size(), 0);
    vector<float> Kds(ids.size(), 0);
    vector<float> positions(ids.size(), 0);
    vector<float> torques(ids.size(), 0);

    // Fetch the previously set Kd for each motor
    for (int i=0; i<ids.size(); i++) {
        int id = ids[i];
        int idx = getIndex(m_ids, id);

        float Kd = m_motors[idx]->Kd;
        if (Kd < 0) {
            cout << "Error! Kd not set for motor " << id << endl;
            exit(1);
        }
        else 
            Kds[i] = Kd;
    }

    // Send the command
    bool success = setCommand(ids, positions, speeds, Kps, Kds, torques);
    return success;
}


/**
 * @brief       Send speed control commands to all motors
 * @param[in]   speeds Goal speeds [rad/s]
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::setSpeeds(vector<float> speeds)
{
    return (setSpeeds(m_ids, speeds));
}

/**
 * @brief       Send speed control commands with FF torques to input motors
 * @param[in]   ids IDs of the motors receiving the commands
 * @param[in]   speeds Goal speeds [rad/s]
 * @param[in]   torques Feedforward torques [Nm]
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::setSpeeds(vector<int> ids, vector<float> speeds, vector<float> torques)
{
    vector<float> Kps(ids.size(), 0);
    vector<float> Kds(ids.size());
    vector<float> positions(ids.size(), 0);

    // Fetch the previously set Kd for each motor
    for (int i=0; i<ids.size(); i++) {
        int id = ids[i];
        int idx = getIndex(m_ids, id);

        float Kd = m_motors[idx]->Kd;
        if (Kd < 0) {
            cout << "Error! Kd not set for motor " << id << endl;
            exit(1);
        }
        else 
            Kds[i] = Kd;
    }

    // Send the command
    bool success = setCommand(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

/**
 * @brief       Send speed control commands with FF torques to all motors
 * @param[in]   speeds Goal speeds [rad/s]
 * @param[in]   torques Feedforward torques [Nm]
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::setSpeeds(vector<float> speeds, vector<float> torques)
{
   return (setSpeeds(m_ids, speeds, torques));
}

/**
 * @brief       Send torque control commands to input motors
 * @param[in]   ids IDs of the motors receiving the commands
 * @param[in]   torques Feedforward torques [Nm]
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::setTorques(vector<int> ids, vector<float> torques)
{
    vector<float> Kps(ids.size(), 0);
    vector<float> Kds(ids.size(), 0);
    vector<float> positions(ids.size(), 0);
    vector<float> speeds(ids.size(), 0);

    // Send the command
    bool success = setCommand(ids, positions, speeds, Kps, Kds, torques);
    return success;
}

/**
 * @brief       Send torque control commands to all motors
 * @param[in]   torques Feedforward torques [Nm]
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::setTorques(vector<float> torques)
{
    return (setTorques(m_ids, torques));
}


/**
 * @brief       Send maintain position command to input motors
 * @param[in]   ids IDs of the motors receiving the commands
 * @param[in]   moving 1 if motors are in motion, 0 if motors static. Default = 1
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::maintainPosition(vector<int> ids, bool moving)
{
    vector<float> fbckPositions(ids.size(), 0);
    bool success = getPositions(ids, fbckPositions, moving);
    if (!success) {
        cout << "Error! Could not get fbck positions to maintain position" << endl;
        return 0;
    }

    // We want to maintain position, but we have no guarantee Kp and Kd are set.
    // Therefore, using some default Kp and Kd values
    vector<float> Kps(ids.size(), 150);
    vector<float> Kds(ids.size(), 2);
    vector<float> speeds(ids.size(), 0);
    vector<float> torques(ids.size(), 0);

    success = setCommand(ids, fbckPositions, speeds, Kps, Kds, torques);
    return success;
}

/**
 * @brief       Send maintain position command to all motors
 * @param[in]   moving 1 if motors are in motion, 0 if motor static. Default = 1
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::maintainPosition(bool moving)
{
    return(maintainPosition(m_ids, moving));
}


/*
 *****************************************************************************
 *                           Getting feedback
 ****************************************************************************/

/**
 * @brief       Get feedback from the input motors
 * @param[in]   ids IDs of the motors receiving the commands
 * @param[out]  fbckPositions Feedback positions [rad]
 * @param[out]  fbckSpeeds Feedback speeds [rad/s]
 * @param[out]  fbckTorques Feedback torques [Nm]
 * @param[out]  fbckTemperatures Feedback temperatures [°C]
 * @param[in]   moving 1 if motors are in motion, 0 if motor static. Default = 1
 * @return      1 if sending the commands was successful, 0 otherwise
 */
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

/**
 * @brief       Get feedback from all motors
 * @param[out]  fbckPositions Feedback positions [rad]
 * @param[out]  fbckSpeeds Feedback speeds [rad/s]
 * @param[out]  fbckTorques Feedback torques [Nm]
 * @param[out]  fbckTemperatures Feedback temperatures [°C]
 * @param[in]   moving 1 if motors are in motion, 0 if motor static. Default = 1
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::getFeedbacks(vector<float>& fbckPositions, vector<float>& fbckSpeeds,
                                vector<float>& fbckTorques, vector<int>& fbckTemperatures, bool moving)
{
    return(getFeedbacks(m_ids, fbckPositions, fbckSpeeds, fbckTorques, fbckTemperatures, moving));
}


/**
 * @brief       Get position feedback from the input motors
 * @param[in]   ids IDs of the motors receiving the commands
 * @param[out]  fbckPositions Feedback positions [rad]
 * @param[in]   moving 1 if motors are in motion, 0 if motor static. Default = 1
 * @return      1 if sending the commands was successful, 0 otherwise
 */
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


/**
 * @brief       Get position feedback from all motors
 * @param[out]  fbckPositions Feedback positions [rad]
 * @param[in]   moving 1 if motors are in motion, 0 if motor static. Default = 1
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::getPositions(vector<float>& fbckPositions, bool moving)
{
    return(getPositions(m_ids, fbckPositions, moving));
}

/**
 * @brief       Get speed feedback from the input motors
 * @param[in]   ids IDs of the motors receiving the commands
 * @param[out]  fbckSpeeds Feedback speeds [rad/s]
 * @param[in]   moving 1 if motors are in motion, 0 if motor static. Default = 1
 * @return      1 if sending the commands was successful, 0 otherwise
 */
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

/**
 * @brief       Get speed feedback from all motors
 * @param[out]  fbckSpeeds Feedback speeds [rad/s]
 * @param[in]   moving 1 if motors are in motion, 0 if motor static. Default = 1
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::getSpeeds(vector<float>& fbckSpeeds, bool moving)
{
    return(getSpeeds(m_ids, fbckSpeeds, moving));
}

/**
 * @brief       Get torque feedback from the input motors
 * @param[in]   ids IDs of the motors receiving the commands
 * @param[out]  fbckTorques Feedback torques [Nm]
 * @param[in]   moving 1 if motors are in motion, 0 if motor static. Default = 1
 * @return      1 if sending the commands was successful, 0 otherwise
 */
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

/**
 * @brief       Get torque feedback from all motors
 * @param[out]  fbckTorques Feedback torques [Nm]
 * @param[in]   moving 1 if motors are in motion, 0 if motor static. Default = 1
 * @return      1 if sending the commands was successful, 0 otherwise
 */
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

/**
 * @brief       Get temperature feedback from all motors
 * @param[out]  fbckTemperatures Feedback temperatures [°C]
 * @param[in]   moving 1 if motors are in motion, 0 if motor static. Default = 1
 * @return      1 if sending the commands was successful, 0 otherwise
 */
bool MotorHandler::getTemperatures(vector<int>& fbckTemperatures, bool moving)
{
    return(getTemperatures(m_ids, fbckTemperatures, moving));
}


}