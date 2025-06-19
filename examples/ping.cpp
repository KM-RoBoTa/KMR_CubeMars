/**
 ********************************************************************************************
 * @file    ex1_impedance.cpp
 * @brief   Example for impedance control
 * @details This is a very simple example to showcase impedance control. \n
 ********************************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 10/2024
 ********************************************************************************************
 */


#include "KMR_CubeMars.hpp"
#include "unistd.h"
#include <cmath>

using namespace std;


// --------------------------------------------------------------------------- //
//                                EDIT HERE 
// --------------------------------------------------------------------------- //

// Id(s) and model(s) of motor(s)
vector<int> ids = {33}; 
int nbrMotors = ids.size();
vector<KMR::CBM::Model> models{KMR::CBM::Model::AK80_8};

const char* can_bus = "can0";
// --------------------------------------------------------------------------- //


int main()
{
    KMR::CBM::MotorHandler motorHandler(ids, can_bus, models);

    return(1);
}
