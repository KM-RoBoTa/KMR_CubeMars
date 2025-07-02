/**
 ******************************************************************************
 * @file            structures.hpp
 * @brief           Different structures used in the library
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 11/2024
 *****************************************************************************
 */

#ifndef KMR_CUBEMARS_STRUCTURES_HPP
#define KMR_CUBEMARS_STRUCTURES_HPP

#include <string>
#include <stdint.h>
#include <vector>
#include <algorithm>

#include "motor_models.hpp"

#define FRAME_LENGTH        8
#define RESPONSE_TIMEOUT    1*1000 // us

namespace KMR::CBM
{

/**
 * @brief   Motor operating modes
 */
enum class OperatingMode {
    MIT
};

/**
 * @brief   Supported models by this library
 */
enum class Model {
    AK10_9,
    AK60_6,
    AK70_10,
    AK80_6,
    AK80_9, 
    AK80_80, 
    AK80_8,
    AK80_64,
    UNDEF_MODEL
};


/**
 * @brief   Structure saving the info of a motor
 * @details The information saved by this structure is parameters depending on the motor model, 
 *          but also all information gotten by the Listener, such as feedback values and receive
 *          flags.
 */
struct Motor {
    int id;
    Model model = Model::UNDEF_MODEL;

    // Parameters depending on motor model
    float minPosition = 0;
    float maxPosition = 0;
    float minSpeed = 0;
    float maxSpeed = 0;
    float minTorque = 0;
    float maxTorque = 0;
    float minKp = 0;
    float maxKp = 0;
    float minKd = 0;
    float maxKd = 0;

    // Motor-specific parameters
    float Kp = -1;
    float Kd = -1;

    // Feedback values
    float fbckPosition = 0;
    float fbckSpeed = 0;
    float fbckTorque = 0;
    int fbckTemperature = 0;

    // CAN listener status flags
    bool fr_fbckReady = 0;

    // Previous frame for getting feedback
    can_frame prev_frame;
    bool f_prevFrame = 0;

    // Constructor
    Motor(int id, Model model)
    {
        this->id = id;
        this->model = model;

        // Save protocol corresponding to the model
        if (model == Model::UNDEF_MODEL) {
            std::cout << "Model for motor " << id << " is undefined!" << std::endl;
            std::cout << std::endl;
            exit(1);
        }
        else {
            ModelParameters parameters;
            switch (model)
            {
            case Model::AK10_9:
                parameters = AK10_9_table();
                break;
            case Model::AK60_6:
                parameters = AK60_6_table();
                break;
            case Model::AK70_10:
                parameters = AK70_10_table();
                break;
            case Model::AK80_6:
                parameters = AK80_6_table();
                break;
            case Model::AK80_80:
                parameters = AK80_80_table();
                break;
            case Model::AK80_8:
                parameters = AK80_8_table();
                break;
            case Model::AK80_9:
                parameters = AK80_9_table();
                break; 
            case Model::AK80_64:
                parameters = AK80_64_table();
                break; 
            default:
                break;
            }

            this->minPosition = parameters.minPosition;
            this->maxPosition = parameters.maxPosition;
            this->minSpeed = parameters.minSpeed;
            this->maxSpeed = parameters.maxSpeed;
            this->minTorque = parameters.minTorque;
            this->maxTorque = parameters.maxTorque;
            this->minKp = parameters.minKp;
            this->maxKp = parameters.maxKp;
            this->minKd = parameters.minKd;
            this->maxKd = parameters.maxKd;
        }
    }
};

}

#endif
