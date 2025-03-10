/**
 ******************************************************************************
 * @file            utils.hpp
 * @brief           Misc. useful functions
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 11/2024
 *****************************************************************************
 */

#ifndef KMR_CUBEMARS_UTILS_HPP
#define KMR_CUBEMARS_UTILS_HPP

#include <vector>
#include <algorithm>
#include <iostream>
#include <unistd.h> // Provides the usleep function

namespace KMR::CBM
{

int getIndex(std::vector<int> v, int k);

float deg2rad(float deg);
float rad2deg(float rad);


timespec time_s();
double get_delta_us(struct timespec t2, struct timespec t1);
timespec convert_to_timespec(timeval tv);
std::string convertToHex(int dec) ;


/**
 * @brief       Saturate the query value between a min and max value
 * @param[in]   min Minimal value
 * @param[in]   max Maximum value
 * @param[in]   val Query value
 * @return      Saturated value
 */
template<typename T>
T saturate(T min, T max, T val)
{
    if (val < min)
        return min;
    else if (val > max)
        return max;
    else
        return val;
}

}

#endif