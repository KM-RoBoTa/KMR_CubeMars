/**
 ******************************************************************************
 * @file            writer.hpp
 * @brief           Definition of the Writer class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 11/2024
 *****************************************************************************
 */

#ifndef KMR_CUBEMARS_WRITER_HPP
#define KMR_CUBEMARS_WRITER_HPP

#include <vector>

#include "../config/structures.hpp"
#include "utils.hpp"

namespace KMR::CBM
{

/**
 * @brief   CAN bus writer
 */
class Writer {
public:
    Writer(std::vector<Motor*> motors, std::vector<int> ids, int s);
    ~Writer();

    // PID settings
    int writeEnterMITMode(int id);
    int writeExitMITMode(int id);
    int writeZeroPosition(int id);
    int writeMITCommand(int id, float pos, float speed, float Kp, float Kd, float torque);
    int writePreviousCommand(int id);

private:
    int m_s; // Socket
    std::vector<int> m_ids;

    std::vector<Motor*> m_motors;
    int m_nbrMotors;

    uint32_t convertSI_to_parameter(float x, float xMin, float xMax, uint bitSize);
};

}

#endif