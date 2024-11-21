/**
 ******************************************************************************
 * @file            motor_handler.hpp
 * @brief           Definition of the MotorHandler class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 11/2024
 *****************************************************************************
 */

#include <iostream>
#include <vector>
#include <cmath>

#include "listener.hpp"
#include "writer.hpp"

#ifndef KMR_CUBEMARS_MOTOR_HANDLER_HPP
#define KMR_CUBEMARS_MOTOR_HANDLER_HPP

namespace KMR::CBM
{

/**
 * @brief   CAN bus listener, running in its own thread
 */
class MotorHandler {
public:
    MotorHandler(std::vector<int> ids, const char* can_bus, std::vector<Model> models);
    ~MotorHandler();

    // ------ Setting parameters ------ //
    void setKps(std::vector<int> ids, std::vector<float> Kps);
    void setKds(std::vector<int> ids, std::vector<float> Kds);

    // ------ Enabling/disabling + 0 setting ------ //
    bool enableMotors();
    bool enableMotors(std::vector<int> ids);
    bool disableMotors();
    bool disableMotors(std::vector<int> ids);

    bool stopMotors(std::vector<int> ids);
    bool stopMotors();
    bool stopMotor(int id);

    bool setZeroPosition(std::vector<int> ids);
    bool setZeroPosition();
    bool setZeroPosition(int id);

    // ------ Control commands ------ //
    bool setImpedance(std::vector<int> ids, std::vector<float> positions, std::vector<float> speeds,
                         std::vector<float> Kps, std::vector<float> Kds, std::vector<float> torques);
    bool setImpedance(std::vector<float> positions, std::vector<float> speeds,
                         std::vector<float> Kps, std::vector<float> Kds, std::vector<float> torques);
  
    bool setPositions(std::vector<int> ids, std::vector<float> positions);
    bool setPositions(std::vector<float> positions);
    bool setPositions(std::vector<int> ids, std::vector<float> positions, std::vector<float> torques);
    bool setPositions(std::vector<float> positions, std::vector<float> torques);

    bool setSpeeds(std::vector<int> ids, std::vector<float> speeds);
    bool setSpeeds(std::vector<float> speeds);
    bool setSpeeds(std::vector<int> ids, std::vector<float> speeds, std::vector<float> torques);
    bool setSpeeds(std::vector<float> speeds, std::vector<float> torques);

    bool setTorques(std::vector<int> ids, std::vector<float> torques);
    bool setTorques(std::vector<float> torques);

    // ------ Feedbacks ------ //
    bool getFeedbacks(std::vector<int> ids, std::vector<float>& fbckPositions, std::vector<float>& fbckSpeeds,
                      std::vector<float>& fbckTorques, std::vector<int>& fbckTemperatures);
    bool getFeedbacks(std::vector<float>& fbckPositions, std::vector<float>& fbckSpeeds,
                      std::vector<float>& fbckTorques, std::vector<int>& fbckTemperatures);
    bool getPositions(std::vector<int> ids, std::vector<float>& fbckPositions);
    bool getPositions(std::vector<float>& fbckPositions);
    bool getSpeeds(std::vector<int> ids, std::vector<float>& fbckSpeeds);
    bool getSpeeds(std::vector<float>& fbckSpeeds);
    bool getTorques(std::vector<int> ids, std::vector<float>& fbckTorques);
    bool getTorques(std::vector<float>& fbckTorques);
    bool getTemperatures(std::vector<int> ids, std::vector<int>& fbckTemperatures);
    bool getTemperatures(std::vector<int>& fbckTemperatures);

private:
    Listener* m_listener = nullptr;
    Writer* m_writer = nullptr;

    std::vector<int> m_ids;
    std::vector<Motor*> m_motors;
    int m_nbrMotors;

    int openSocket(const char* can_bus);
    void pingMotors();
};

}

#endif