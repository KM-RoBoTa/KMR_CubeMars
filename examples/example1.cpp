#include "motor_handler.hpp"
#include "unistd.h"
#include <cmath>

using namespace std;

int main()
{
    vector<Model> models(1, Model::AK60_6);
    vector<int> ids(1,1);

    const char* can_bus = "can0";
    MotorHandler motorHandler(ids, can_bus, models);

    sleep(1);
    cout << "Going to MIT mode" << endl;
    motorHandler.enterMITMode();

    sleep(5);

    cout << "Exiting MIT mode" << endl;
    motorHandler.exitMITMode();
    sleep(1);

    return(1);
}