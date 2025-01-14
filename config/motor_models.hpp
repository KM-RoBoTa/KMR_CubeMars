/**
 ******************************************************************************
 * @file            motor_models.hpp
 * @brief           Configuration file of AK motors
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 11/2024
 *****************************************************************************
 */

#ifndef KMR_CUBEMARS_MOTOR_MODELS_HPP
#define KMR_CUBEMARS_MOTOR_MODELS_HPP

#include "structures.hpp"

namespace KMR::CBM
{

/**
 * @brief   Base control table of CubeMars motors, inherited by specific models
 */
struct ModelParameters {
    float minPosition = -12.5;
    float maxPosition = 12.5;
    float minSpeed = 0;
    float maxSpeed = 0;
    float minTorque = 0;
    float maxTorque = 0;
    float minKp = 0;
    float maxKp = 500;
    float minKd = 0;
    float maxKd = 5;
};


/**
 * @brief   Control table of AK10-9 motors
 */
struct AK10_9_table : ModelParameters {
    AK10_9_table ()
    {
        minSpeed  = -50;
        maxSpeed  = +50;
        minTorque = -65;
        maxTorque = +65;
    }
};

/**
 * @brief   Control table of AK60-6 motors
 */
struct AK60_6_table : ModelParameters {
    AK60_6_table ()
    {
        minSpeed  = -45;
        maxSpeed  = +45;
        minTorque = -15;
        maxTorque = +15;
    }
};

/**
 * @brief   Control table of AK70-10 motors
 */
struct AK70_10_table : ModelParameters {
    AK70_10_table ()
    {
        minSpeed  = -50;
        maxSpeed  = +50;
        minTorque = -25;
        maxTorque = +25;
    }
};

/**
 * @brief   Control table of AK80-6 motors
 */
struct AK80_6_table : ModelParameters {
    AK80_6_table ()
    {
        minSpeed  = -76;
        maxSpeed  = +76;
        minTorque = -12;
        maxTorque = +12;
    }
};

/**
 * @brief   Control table of AK80-9 motors
 */
struct AK80_9_table : ModelParameters {
    AK80_9_table ()
    {
        minSpeed  = -50;
        maxSpeed  = +50;
        minTorque = -18;
        maxTorque = +18;
    }
};

/**
 * @brief   Control table of AK80-80 motors
 */
struct AK80_80_table : ModelParameters {
    AK80_80_table ()
    {
        minSpeed  = -8;
        maxSpeed  = +8;
        minTorque = -144;
        maxTorque = +144;
    }
};

/**
 * @brief   Control table of AK80-8 motors
 */
struct AK80_8_table : ModelParameters {
    AK80_8_table ()
    {
        minSpeed  = -37.5;
        maxSpeed  = +37.5;
        minTorque = -32;
        maxTorque = +32;
    }
};

}

#endif