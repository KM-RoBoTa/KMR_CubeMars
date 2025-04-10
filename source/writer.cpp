/**
 ******************************************************************************
 * @file            writer.cpp
 * @brief           Methods of the Writer class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 11/2024
 *****************************************************************************
 */

#include <linux/can.h>
#include <iostream>
#include <unistd.h>
#include <cmath>

#include "writer.hpp"

using namespace std;

namespace KMR::CBM
{

/**
 * @brief       Constructor for the CAN writer
 * @param[in]   motors List of all CubeMars motors
 * @param[in]   ids IDs of all motors
 * @param[in]   s CAN socket
 */
Writer::Writer(vector<Motor*> motors, vector<int> ids, int s)
{
    m_s = s;
    m_motors = motors;
	m_nbrMotors = motors.size();
    m_ids = ids;
}

/**
 * @brief	Class destructor
 */
Writer::~Writer()
{

}


/**
 * 	@brief 		Convert input value from SI units to motor parameter
 * 	@param[in] 	x Value to be converted to a parameter
 * 	@param[in] 	xMin Minimum value that the input value can take 
 * 	@param[in] 	xMax Maximum value that the input value can take 
 * 	@param[in] 	bitSize Number of bits encoding the parameter
 * 	@return 	Input value converted to uin32t_t
 */ 
uint32_t Writer::convertSI_to_parameter(float x, float xMin, float xMax, uint bitSize)
{
    float span = xMax - xMin;
    x = saturate(xMin, xMax, x);
    
    uint32_t parameter = (uint32_t) ( (x-xMin)*((float)(1<<bitSize)-1)/span );
    return parameter;
}


/*
 *****************************************************************************
 *                               Motor infos
 ****************************************************************************/

/**
 * 	@brief 		Send the "enter MIT mode" command to the input motor
 * 	@param[in] 	id ID of the target motor
 * 	@return 	Number of bytes sent to the CAN bus. Equal to -1 if sending failed
 */ 
int Writer::writeEnterMITMode(int id)
{
    struct can_frame frame;
    frame.can_id = id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFC;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;        
}

/**
 * 	@brief 		Send the "exit MIT mode" command to the input motor
 * 	@param[in] 	id ID of the target motor
 * 	@return 	Number of bytes sent to the CAN bus. Equal to -1 if sending failed
 */ 
int Writer::writeExitMITMode(int id)
{
    struct can_frame frame;
    frame.can_id = id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFD;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;        
}

/**
 * 	@brief 		Send the "set zero position" command to the input motor
 * 	@param[in] 	id ID of the target motor
 * 	@return 	Number of bytes sent to the CAN bus. Equal to -1 if sending failed
 */ 
int Writer::writeZeroPosition(int id)
{
    struct can_frame frame;
    frame.can_id = id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFE;

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;       
}

/**
 * 	@brief 		Send the MIT control command to the input motor
 * 	@param[in] 	id ID of the target motor
 *  @param[in]  position Goal position [rad]
 *  @param[in]  speed Goal speed [rad/s]
 *  @param[in]  Kp Factor for position difference
 *  @param[in]  Kd Factor for speeds difference
 *  @param[in]  torque Goal torque [Nm]
 * 	@return 	Number of bytes sent to the CAN bus. Equal to -1 if sending failed
 */ 
int Writer::writeMITCommand(int id, float position, float speed, float Kp, float Kd, float torque)
{
    // Change the signs from our custom reference to CubeMars' driver definition
    position = -position;
    speed = -speed;
    torque = -torque;

    // Extract min/max parameters from the query motor
    int idx = getIndex(m_ids, id);
    float minPosition = m_motors[idx]->minPosition;
    float maxPosition = m_motors[idx]->maxPosition;
    float minSpeed = m_motors[idx]->minSpeed;
    float maxSpeed = m_motors[idx]->maxSpeed;
    float minKp = m_motors[idx]->minKp;
    float maxKp = m_motors[idx]->maxKp;
    float minKd = m_motors[idx]->minKd;
    float maxKd = m_motors[idx]->maxKd;
    float minTorque = m_motors[idx]->minTorque;
    float maxTorque = m_motors[idx]->maxTorque;

    // Convert to parameters (includes saturation)
    uint32_t positionParam = convertSI_to_parameter(position, minPosition, maxPosition, 16);
    uint32_t speedParam = convertSI_to_parameter(speed, minSpeed, maxSpeed, 12);
    uint32_t KpParam = convertSI_to_parameter(Kp, minKp, maxKp, 12);
    uint32_t KdParam = convertSI_to_parameter(Kd, minKd, maxKd, 12);
    uint32_t torqueParam = convertSI_to_parameter(torque, minTorque, maxTorque, 12);

    // Create CAN packet
    struct can_frame frame;
    frame.can_id = id;
    frame.len = FRAME_LENGTH;
    frame.data[0] =  positionParam>>8;
    frame.data[1] =  positionParam&0xFF;
    frame.data[2] =  speedParam>>4;
    frame.data[3] =  ((speedParam&0xF)<<4) | (KpParam>>8);
    frame.data[4] = KpParam&0xFF;
    frame.data[5] = KdParam>>4;
    frame.data[6] = ((KdParam&0xF)<<4)| (torqueParam>>8);
    frame.data[7] = torqueParam&0xFF;

    // Print the packet
    /*cout << endl;
    cout << "ID: " << frame.can_id << endl;
    for (int i=0; i<8; i++)
        cout << "Data " << i << ": 0x" << convertToHex(frame.data[i]) << endl;*/

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));

    // Save this packet to reuse it if feedback reading required 
    if (nbytes > 0) {
        m_motors[idx]->f_prevFrame = 1;
        m_motors[idx]->prev_frame = frame;
    }
 
    return nbytes;           
}


/**
 * 	@brief 		Resend previous command
 *  @note       This is used (and required) to get feedback during movement
 * 	@param[in] 	id ID of the target motor
 * 	@return 	Number of bytes sent to the CAN bus. Equal to -1 if sending failed
 */ 
int Writer::writePreviousCommand(int id)
{
    int idx = getIndex(m_ids, id);
    can_frame frame;
    if (m_motors[idx]->f_prevFrame) {
        frame = m_motors[idx]->prev_frame;
        m_motors[idx]->f_prevFrame = 0;
    }
    else {
        cout << "Error! The motor " << id << " has no previous frame to be resent" << endl;
        exit(1);
    }

    // Send frame
    int nbytes = -1;
    nbytes = write(m_s, &frame, sizeof(can_frame));
 
    return nbytes;          
}

}