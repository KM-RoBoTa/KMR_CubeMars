/**
 ******************************************************************************
 * @file            writer.hpp
 * @brief           Header for the writer.cpp file
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 09/2024
 * @authors  kamilo.melo@km-robota.com, 09/2024
 ******************************************************************************
 */

#ifndef KMR_CUBEMARS_WRITER_HPP
#define KMR_CUBEMARS_WRITER_HPP

#include <vector>

#include "../config/structures.hpp"
#include "utils.hpp"

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
    int writeSetZeroPosition(int id);
    int writeMITCommand(int id, float pos, float speed, float Kp, float Kd, float Tff);

private:
    int m_s; // Socket
    std::vector<int> m_ids;

    std::vector<Motor*> m_motors;
    int m_nbrMotors;
};


#endif