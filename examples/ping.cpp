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
vector<int> ids = {1, 3, 4}; 
int nbrMotors = ids.size();
vector<KMR::CBM::Model> models{KMR::CBM::Model::AK60_6, KMR::CBM::Model::AK60_6, KMR::CBM::Model::AK60_6};

const char* can_bus = "can0";
// --------------------------------------------------------------------------- //


int main()
{
    KMR::CBM::MotorHandler motorHandler(ids, can_bus, models);
    motorHandler.enableMotors(); // debug

    return(1);
}
