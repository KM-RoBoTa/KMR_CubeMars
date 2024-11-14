/**
 ******************************************************************************
 * @file            motor_handler.hpp
 * @brief           Header for the motor_handler.cpp file
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ******************************************************************************
 */

#include <iostream>
#include <vector>
#include <cmath>

//#include "listener.hpp"
#include "writer.hpp"

#ifndef KMR_CUBEMARS_MOTOR_HANDLER_HPP
#define KMR_CUBEMARS_MOTOR_HANDLER_HPP

/**
 * @brief   CAN bus listener, running in its own thread
 */
class MotorHandler {
public:
    MotorHandler(std::vector<int> ids, const char* can_bus, std::vector<Model> models);
    ~MotorHandler();

    bool enterMITMode();
    bool enterMITMode(std::vector<int> ids);
    bool exitMITMode();
    bool exitMITMode(std::vector<int> ids);
    // 0 position
    bool writeMITCommand();

private:
    //Listener* m_listener = nullptr;
    Writer* m_writer = nullptr;

    std::vector<int> m_ids;
    std::vector<Motor*> m_motors;
    int m_nbrMotors;

    int openSocket(const char* can_bus);
    

};

#endif