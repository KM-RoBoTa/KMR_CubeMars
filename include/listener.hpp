/**
 ******************************************************************************
 * @file            listener.hpp
 * @brief           Definition of the Listener class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 11/2024
 *****************************************************************************
 */

#ifndef KMR_CUBEMARS_LISTENER_HPP
#define KMR_CUBEMARS_LISTENER_HPP

#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <linux/can.h>

#include "../config/structures.hpp"
#include "utils.hpp"

namespace KMR::CBM
{

/**
 * @brief   CAN bus listener, running in its own thread
 */
class Listener {
public:
    Listener(std::vector<Motor*> motors, std::vector<int> ids, int s);
    ~Listener();

    bool fbckReceived(int id);
    bool getFeedbacks(int id, float& fbckPosition, float& fbckSpeed,
                      float& fbckTorque, int& fbckTemperature);

private:
    // Thread
    bool m_stopThread = 0;
    std::thread m_thread;
    std::mutex m_mutex;

    std::vector<Motor*> m_motors;
    int m_nbrMotors;
    std::vector<int> m_ids;

    int listenerLoop(int s);
    void parseFrame(can_frame frame);
    float convertParameter_to_SI(int x, float xMin, float xMax, int bitSize);
};

}

#endif