#include "KMR_CubeMars.hpp"
#include "unistd.h"
#include <cmath>

#define CTRL_PERIOD_US  5000

using namespace std;

int main()
{
    vector<KMR::CBM::Model> models(1, KMR::CBM::Model::AK60_6);
    vector<int> ids(1,1);

    const char* can_bus = "can0";
    KMR::CBM::MotorHandler motorHandler(ids, can_bus, models);

    sleep(1);
    cout << "Setting 0" << endl;
    motorHandler.setZeroPosition(ids);
    sleep(1);

    sleep(3);
    cout << "Going to MIT mode" << endl;
    motorHandler.enableMotors();

    cout << "Sending the command" << endl;
    float goalPosition = M_PI/2;
    //vector<float> positions(1, goalPosition);
    vector<float> positions(1, 0);
    //vector<float> speeds(1, -2*M_PI/3);
    vector<float> speeds(1, 0);
    vector<float> torques(1, -0.5);
    vector<float> Kps(1, 0);
    vector<float> Kds(1,0);

    vector<float> fbckPositions(1), fbckSpeeds(1), fbckTorques(1);
    vector<int> fbckTemperatures(1);

    cout << endl << endl << "Writing command" << endl;
    motorHandler.setCommand(ids, positions, speeds, Kps, Kds, torques);
    
    int ctr = 0;
    const int maxCtr = 1000; 
    while(ctr < maxCtr) {
        timespec start = KMR::CBM::time_s();

        motorHandler.getFeedbacks(ids, fbckPositions, fbckSpeeds, fbckTorques, fbckTemperatures);
        cout << "Pos: " << fbckPositions[0] << " rad, speed: " << fbckSpeeds[0] <<
        " rad/s, torque: " << fbckTorques[0] << " Nm, temp: " << fbckTemperatures[0] << " °C" << endl;

        //float speedC = (goalPosition - fbckPositions[0])/(CTRL_PERIOD_US/1000000.0);
        //speeds[0] = speedC;


        motorHandler.setCommand(ids, positions, speeds, Kps, Kds, torques);
        ctr++;

        timespec end = KMR::CBM::time_s();
        double elapsed = KMR::CBM::get_delta_us(end, start);
        double toSleep_us = CTRL_PERIOD_US-elapsed;
        if (toSleep_us < 0)
            toSleep_us = 0;
        usleep(toSleep_us);
    }

    return(1);
}