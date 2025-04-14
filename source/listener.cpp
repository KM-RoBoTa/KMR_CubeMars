/**
 ******************************************************************************
 * @file            listener.cpp
 * @brief           Methods of the Listener class
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
#include <sys/ioctl.h>
#include <cstring>
#include <net/if.h> // network devices
#include <sstream> // Convert dec-hex

#include <sys/mman.h>

#include "listener.hpp"

using namespace std;

namespace KMR::CBM
{

struct Wrap {
    Listener* ins; 	// ptr to instance of Listener
    int* socket;   		// socket

    Wrap(Listener* ins, int* socket) {
		this->socket = socket;
		this->ins = ins;
	}
};


/**
 * @brief       Start the CAN bus listener
 * @details		The listener works in its own thread, created on constructor call. 
 * 				The thread is safely stopped on destructor call
 * @param[in]	motors List of CubeMars motors models
 * @param[in]	ids IDs of the motors
 * @param[in]   s Socket
 */
Listener::Listener(vector<Motor*> motors, vector<int> ids, int s)
{
	m_motors = motors;
	m_nbrMotors = motors.size();
    m_stopThread = false;

	// Wrap the arguments
	Wrap* wrap = new Wrap(this, &s);

	/* Lock memory */
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
			printf("mlockall failed: %m\n");
			exit(-2);
	}

	// Create RT thread
	pthread_attr_t tattr;
	sched_param param;

	/* initialized with default attributes */
	int ret = pthread_attr_init (&tattr);
	if (ret) {
		cout << "init pthread attributes failed" << endl;
		exit(1);
	}

	/* Set a specific stack size  */
	ret = pthread_attr_setstacksize(&tattr, PTHREAD_STACK_MIN);
	if (ret) {
		cout << "pthread setstacksize failed" << endl;
		exit(1);
	}

	/* Set scheduler policy and priority of pthread */
	ret = pthread_attr_setschedpolicy(&tattr, SCHED_RR);
	if (ret) {
		cout << "pthread setschedpolicy failed" << endl;
		exit(1);
	}

	/* safe to get existing scheduling param */
	//ret = pthread_attr_getschedparam (&tattr, &param);
	//cout << "Default prio: " << param.sched_priority << endl;

	/* set the priority; others are unchanged */
	int newprio = 95;
	param.sched_priority = newprio;

	/* setting the new scheduling param */
	ret = pthread_attr_setschedparam(&tattr, &param);
	if (ret) {
		cout << "pthread setschedparam failed" << endl;
		exit(1);
	}

	/* Use scheduling parameters of attr */
	// LOL: https://stackoverflow.com/questions/3206814/pthreads-with-real-time-priority
	ret = pthread_attr_setinheritsched(&tattr, PTHREAD_EXPLICIT_SCHED);
	if (ret) {
		cout << "pthread setinheritsched failed" << endl;
		exit(1);
	}

	/* with new priority specified */
	//ret = pthread_create (&m_thread, NULL, &Listener::listenerLoop_helper, wrap); 
	ret = pthread_create (&m_thread, &tattr, &Listener::listenerLoop_helper, wrap); 
	if (ret) {
		cout << "ret = " << ret << endl;
		if (ret == EPERM)
			cout << "Need root perm to create pthread" << endl;
		else if (ret == EAGAIN)
			cout << "Insufficient resources to create another thread" << endl;
		else if (ret == EINVAL)
			cout << "Invalid settings in attr" << endl;
		else 
			cout << "pthread create failed" << endl;
		exit(1);
	}

	m_ids = ids;

    cout << "Creating the CAN listener's thread..." << endl;

	sleep(2);  // Make sure the Listener thread started before deleting wrap
	delete wrap;
}

/**
 * @brief	Class destructor. Takes care of safely stopping the thread
 */
Listener::~Listener()
{
	// Change the internal boolean to get the thread out of its loop function
	{
		pthread_mutex_lock( &m_mutex );
		m_stopThread = true;	
		pthread_mutex_unlock( &m_mutex );
	}	
    
	// Block the main thread until the Listener thread finishes
	pthread_join(m_thread, NULL);

	cout << "Listener thread safely stopped" << endl;
}

/**
 * @brief       Function executed by the Listener thread
 * @param[in]   s Socket
 * @retval      1 when the thread is finished
 */
void Listener::listenerLoop(int s)
{
	bool stopThread = 0;

	cout << "Starting CAN bus monitoring" << endl;
    while(!stopThread) {

        struct can_frame frame;
        int nbytes = read(s, &frame, sizeof(can_frame));  // Usually takes ~4us, rarely jumping to 26 us

        if (nbytes > 0)
			parseFrame(frame);

		//usleep(10000);

		// Thread sleep for scheduling
		//std::this_thread::sleep_for(chrono::microseconds(10));  // 50 at start
		//{
		//	pthread_mutex_lock( &m_mutex );
		//	stopThread = m_stopThread;	
		//	pthread_mutex_unlock(&m_mutex );
		//}

		pthread_mutex_lock( &m_mutex );
		stopThread = m_stopThread;	
		pthread_mutex_unlock(&m_mutex );
    }
}


void* Listener::listenerLoop_helper(void* wrap)
{
	// Convert wrap from void* to Wrap* = cast args into a meaningful pointer type that we can use
	Wrap* w = static_cast<Wrap*>(wrap); 

	// Call the real method we want to run
	int s = *(w->socket);
	w->ins->listenerLoop(s);

	return NULL;
}


/**
 * 	@brief 		Convert a parameter received from motors to SI units
 * 	@param[in] 	x Value to be converted to SI
 * 	@param[in] 	xMin Minimum value that the converted parameter can take 
 * 	@param[in] 	xMax Maximum value that the converted parameter can take 
 * 	@param[in] 	bitSize Number of bits encoding the parameter
 * 	@return 	Parameter converted to SI units
 */ 
float Listener::convertParameter_to_SI(int x, float xMin, float xMax, int bitSize)
{
	float span = xMax - xMin;

	float value = x * span/(float)((1<<bitSize)-1) + xMin;
	return value;
}


/**
 * 	@brief 		Parse a received frame
 * 	@param[in] 	frame CAN frame received on the bus
 */ 
void Listener::parseFrame(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.data[0];
	int idx = getIndex(m_ids, id);	

	// Parse data
	int positionParameter = (frame.data[1] << 8) | frame.data[2];
	int speedParameter = (frame.data[3] << 4) | (frame.data[4] >> 4);
	int torqueParameter = ((frame.data[4]&0xF) << 8) | frame.data[5];
	int temperatureParameter = frame.data[6];

    // Extract min/max parameters from the query moto
    float minPosition = m_motors[idx]->minPosition;
    float maxPosition = m_motors[idx]->maxPosition;
    float minSpeed = m_motors[idx]->minSpeed;
    float maxSpeed = m_motors[idx]->maxSpeed;
    float minTorque = m_motors[idx]->minTorque;
    float maxTorque = m_motors[idx]->maxTorque; 

	// Convert parameters to SI
	float position = convertParameter_to_SI(positionParameter, minPosition, maxPosition, 16);
	float speed = convertParameter_to_SI(speedParameter, minSpeed, maxSpeed, 12);
	float torque = convertParameter_to_SI(torqueParameter, minTorque, maxTorque, 12);
	int temperature = temperatureParameter;

	// Adjust signs for our reference
	position = -position;
	speed = -speed;
	torque = -torque;

	// Save to internal structure
	pthread_mutex_lock( &m_mutex );
	m_motors[idx]->fbckPosition = position;
	m_motors[idx]->fbckSpeed = speed;
	m_motors[idx]->fbckTorque = torque;
	m_motors[idx]->fbckTemperature = temperature;

	m_motors[idx]->fr_fbckReady = 1;
	pthread_mutex_unlock( &m_mutex );
}


/**
 * @brief       Check if the query motor sent the response to a previous command
 * @param[in]   id ID of the query motor
 * @retval      1 if the motor answered, 0 if not (timeout)
 */
bool Listener::fbckReceived(int id)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Receiving feedback for motor " << id << " timed out!" << endl;
			break;
		}		

		pthread_mutex_lock( &m_mutex );
		available = m_motors[idx]->fr_fbckReady;

		// Clear update flag
		m_motors[idx]->fr_fbckReady = 0;
		pthread_mutex_unlock( &m_mutex );
	}

	return available;		
}


/**
 * @brief       Get the feedbacks from a query motor, if it was received
 * @param[in]   id ID of the query motor
 * @param[out]	fbckPosition Feedback motor position [rad]
 * @param[out]	fbckSpeed Feedback motor speed [rad/s]
 * @param[out]	fbckTorque Feedback torque [Nm]
 * @param[out]	fbckTemperature Feedback temperature [°C]
 * @retval      1 if the motor answered (= valid fbck), 0 if not (timeout)
 */
bool Listener::getFeedbacks(int id, float& fbckPosition, float& fbckSpeed,
							float& fbckTorque, int& fbckTemperature)
{
	int idx = getIndex(m_ids, id);
	bool available = 0;

	timespec start = time_s();
	while (available != 1) {
		// Check for timeout
		timespec end = time_s();
		double elapsed = get_delta_us(end, start);
		if (elapsed > RESPONSE_TIMEOUT) {
			cout << "[TIMEOUT] Receiving feedback for motor " << id << " timed out!" << endl;
			break;
		}		

		pthread_mutex_lock( &m_mutex );
		available = m_motors[idx]->fr_fbckReady;
		fbckPosition = m_motors[idx]->fbckPosition;
		fbckSpeed = m_motors[idx]->fbckSpeed;
		fbckTorque = m_motors[idx]->fbckTorque;
		fbckTemperature = m_motors[idx]->fbckTemperature;

		// Clear update flag
		m_motors[idx]->fr_fbckReady = 0;

		pthread_mutex_unlock( &m_mutex );
	}

	return available;	
}

}