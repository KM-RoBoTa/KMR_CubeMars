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

#include "listener.hpp"
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

    // Setting parameters
    void setKps(std::vector<int> ids, std::vector<float> Kps);
    void setKds(std::vector<int> ids, std::vector<float> Kds);

    bool enterMITMode();
    bool enterMITMode(std::vector<int> ids);
    bool exitMITMode();
    bool exitMITMode(std::vector<int> ids);
    bool setZeroPosition(std::vector<int> ids);
    bool setZeroPosition();
    bool setZeroPosition(int id);


    bool sendImpedanceCommand(std::vector<int> ids, std::vector<float> positions, std::vector<float> speeds,
                         std::vector<float> Kps, std::vector<float> Kds, std::vector<float> torques);
    bool sendImpedanceCommand(std::vector<float> positions, std::vector<float> speeds,
                         std::vector<float> Kps, std::vector<float> Kds, std::vector<float> torques);

    bool getFeedbacks(std::vector<int> ids, std::vector<float>& fbckPositions, std::vector<float>& fbckSpeeds,
                      std::vector<float>& fbckTorques, std::vector<int>& fbckTemperatures);
    bool getFeedbacks(std::vector<float>& fbckPositions, std::vector<float>& fbckSpeeds,
                      std::vector<float>& fbckTorques, std::vector<int>& fbckTemperatures);
  
    bool sendPosition(std::vector<int> ids, std::vector<float> positions);
    bool sendPosition(std::vector<float> positions);
    bool sendPosition(std::vector<int> ids, std::vector<float> positions, std::vector<float> torques);
    bool sendPosition(std::vector<float> positions, std::vector<float> torques);

    bool sendSpeed(std::vector<int> ids, std::vector<float> speeds);
    bool sendSpeed(std::vector<float> speeds);
    bool sendSpeed(std::vector<int> ids, std::vector<float> speeds, std::vector<float> torques);
    bool sendSpeed(std::vector<float> speeds, std::vector<float> torques);

    bool sendTorque(std::vector<int> ids, std::vector<float> torques);
    bool sendTorque(std::vector<float> torques);

private:
    Listener* m_listener = nullptr;
    Writer* m_writer = nullptr;

    std::vector<int> m_ids;
    std::vector<Motor*> m_motors;
    int m_nbrMotors;

    int openSocket(const char* can_bus);
    

};

#endif