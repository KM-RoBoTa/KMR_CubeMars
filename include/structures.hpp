/**
 ******************************************************************************
 * @file            structures.hpp
 * @brief           Structures used in the library
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 11/2024
 *****************************************************************************
 */

#include <iostream>
#include <vector>


#pragma once

namespace KMR::CBM
{

/**
 * @brief 
 */
struct InputPkg {
    std::vector<float> positions;
    std::vector<float> speeds;
    std::vector<float> Kps;
    std::vector<float> Kds;
    std::vector<float> torques;

    InputPkg(){};

    InputPkg(std::vector<float> positions, std::vector<float> speeds,
            std::vector<float> Kps, std::vector<float> Kds, std::vector<float> torques) {
        this->positions = positions;
        this->speeds = speeds;
        this->Kps = Kps;
        this->Kds = Kds;
        this->torques = torques;
    };
};

struct FeedbackPkg {
    std::vector<float> fbckPositions;
    std::vector<float> fbckSpeeds;
    std::vector<float> fbckTorques;
    std::vector<int> fbckTemperatures;

    FeedbackPkg(){};
    FeedbackPkg(std::vector<float> fbckPositions, std::vector<float> fbckSpeeds, 
                std::vector<float> fbckTorques, std::vector<int> fbckTemperatures) {
        this->fbckPositions = fbckPositions;
        this->fbckSpeeds = fbckSpeeds;
        this->fbckTorques = fbckTorques;
        this->fbckTemperatures = fbckTemperatures;
    };
    FeedbackPkg(int size) {
        fbckPositions = std::vector<float>(size, 0);
        fbckSpeeds = std::vector<float>(size, 0);
        fbckTorques = std::vector<float>(size, 0);
        fbckTemperatures = std::vector<int>(size, 0);
    };
};

}