/**
 ******************************************************************************
 * @file            utils.cpp
 * @brief           Misc. useful functions
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 11/2024
 *****************************************************************************
 */

#include "utils.hpp"
#include <cmath>

#include <sstream> // Convert dec-hex
#include <cstring>

using namespace std;


namespace KMR::CBM
{

/**
 * @brief       Return the index of an element k
 * @param[in]   v Query vector
 * @param[in]   k Element to be looked for in the vector
 * @return      Index of k in the vector. Equal to -1 if element not in vector
 */
int getIndex(std::vector<int> v, int k) 
{ 
    auto it = std::find(v.begin(), v.end(), k); 
    int index = -1;
  
    // If element was found 
    if (it != v.end())  
        index = it - v.begin(); 
    return index;
} 

/**
 * @brief       Convert an angle from degrees to radians
 * @param[in]   deg Input angle [°]
 * @return      Angle in radians
 */
float deg2rad(float deg)
{
    return (deg * M_PI / 180.0);
}

/**
 * @brief       Convert an angle from radians to degrees
 * @param[in]   rad Input angle [rad]
 * @return      Angle in degrees
 */
float rad2deg(float rad)
{
    return (rad * 180 / M_PI);
}


/**
 * @brief   Get current time structure
 * @return  Current time structure
 */
timespec time_s()
{
    struct timespec real_time;

    if (clock_gettime(CLOCK_REALTIME, &real_time) == -1 )
    {
        perror("clock gettime");
        exit( EXIT_FAILURE );
    }

    return real_time;
}

/**
 * @brief       Get elapsed time, in microseconds
 * @param[in]   t2 End time structure (gotten with time_s)
 * @param[in]   t1 Start time structure (gotten with time_s)
 * @return      Elapsed time between t1 and t2, in us
 */
double get_delta_us(struct timespec t2, struct timespec t1)
{
    struct timespec td;
    td.tv_nsec = t2.tv_nsec - t1.tv_nsec;
    td.tv_sec  = t2.tv_sec - t1.tv_sec;
    if (td.tv_sec > 0 && td.tv_nsec < 0)
    {
        td.tv_nsec += 1000000000;
        td.tv_sec--;
    }

    return(td.tv_sec*1000000 + td.tv_nsec/1000);
}

/**
 * @brief       Convert the timeval input to a timespec structure
 * @param[in]   tv Time encoded in a timeval structure
 * @return      Time converted into a timespec structure
 */
timespec convert_to_timespec(timeval tv)
{
    timespec td;

    td.tv_sec = tv.tv_sec;
    td.tv_nsec = tv.tv_usec*1000;
    return td;
}

/**
 * @brief       Convert a decimal number into hex, gotten in a string
 * @param[in]   dec Query decimal value
 * @return      String containing the value in hex
 */
std::string convertToHex(int dec) 
{
    std::stringstream ss;
    ss << std::hex << dec; // int decimal_value
    std::string res ( ss.str() );

    return res;
}

}