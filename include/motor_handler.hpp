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
 * @brief   Highest-level class that manages all communication with the motors
 */
class MotorHandler {
public:
    MotorHandler(std::vector<int> ids, const char* can_bus, std::vector<Model> models);
    ~MotorHandler();

    // ------ Setting parameters ------ //

    void setKps(std::vector<int> ids, std::vector<float> Kps);
    void setKds(std::vector<int> ids, std::vector<float> Kds);

    // ------ Torque disabling + 0 setting ------ //

    bool stopMotors(std::vector<int> ids);
    bool stopMotors();
    bool stopMotor(int id);

    bool setZeroPosition(std::vector<int> ids);
    bool setZeroPosition();
    bool setZeroPosition(int id);

    // ------ Control commands ------ //

    bool setCommand(std::vector<int> ids, std::vector<float> positions, std::vector<float> speeds,
                         std::vector<float> Kps, std::vector<float> Kds, std::vector<float> torques);
    bool setCommand(std::vector<float> positions, std::vector<float> speeds,
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

    bool maintainPosition(std::vector<int> ids, bool moving = 1);
    bool maintainPosition(bool moving = 1);

    // ------ Feedbacks ------ //

    bool getFeedbacks(std::vector<int> ids, std::vector<float>& fbckPositions, std::vector<float>& fbckSpeeds,
                      std::vector<float>& fbckTorques, std::vector<int>& fbckTemperatures, bool moving = 1);
    bool getFeedbacks(std::vector<float>& fbckPositions, std::vector<float>& fbckSpeeds,
                      std::vector<float>& fbckTorques, std::vector<int>& fbckTemperatures, bool moving = 1);
    bool getPositions(std::vector<int> ids, std::vector<float>& fbckPositions, bool moving = 1);
    bool getPositions(std::vector<float>& fbckPositions, bool moving = 1);
    bool getSpeeds(std::vector<int> ids, std::vector<float>& fbckSpeeds, bool moving = 1);
    bool getSpeeds(std::vector<float>& fbckSpeeds, bool moving = 1);
    bool getTorques(std::vector<int> ids, std::vector<float>& fbckTorques, bool moving = 1);
    bool getTorques(std::vector<float>& fbckTorques, bool moving = 1);
    bool getTemperatures(std::vector<int> ids, std::vector<int>& fbckTemperatures, bool moving = 1);
    bool getTemperatures(std::vector<int>& fbckTemperatures, bool moving = 1);

private:
    Listener* m_listener = nullptr;
    Writer* m_writer = nullptr;

    std::vector<int> m_ids;
    std::vector<Motor*> m_motors;
    int m_nbrMotors;

    int openSocket(const char* can_bus);
    void pingMotors();

    // ------ Enabling/disabling = setting/exiting MIT mode ------ //

    bool enableMotors();
    bool enableMotors(std::vector<int> ids);
    bool disableMotors();
    bool disableMotors(std::vector<int> ids);
};

}

#endif