// Standard libraries
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <cmath>

#include <sys/socket.h>
#include <linux/can.h>
#include <sys/ioctl.h>
#include <cstring>
#include <net/if.h> // network devices

#include <sstream> // Convert dec-hex

#include "KMR_CubeMars.hpp"


#include <linux/sockios.h>

#define FRAME_LENGTH 8
const int SOCKET_TIMEOUT_US = 5*1000; // 30ms in us

// --------------------------------------------------------------------------- //
//                                EDIT HERE 
// --------------------------------------------------------------------------- //

// Id(s) and model(s) of motor(s)
int id = 1;
const char* can_bus = "can0";
// --------------------------------------------------------------------------- //


struct sockaddr_can addr;
struct ifreq ifr;
int s;


// Fcts
int openSocket();
void enterMITMode(int id);
void exitMITMode(int id);
void setZeroPosition(int id);
void stopMotor(int id);


// NOTE: Stop motors for AK60-6
// id = id
// 7F
// FF
// 7F
// F0 
// 00
// 00
// 07
// FF



using namespace std;


int main()
{
    s = openSocket();

    enterMITMode(id);

    sleep(3);

    /*setZeroPosition(id);

    sleep(3);

    for (int i=0; i<1000; i++) {
        stopMotor(id);
        usleep(10*1000);
    }*/

    exitMITMode(id);

    sleep(3);

    // Close the socket
    close(s);

    return 1;
}

int openSocket()
{
    // Create a CAN socket
    //int s = socket(PF_CAN, SOCK_RAW|SOCK_NONBLOCK, CAN_RAW);
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        cout << "Error creating the socket! Exiting" << endl;
        exit(1);
    }
    else   
        cout << "Socket created successfully" << endl;

    // Set socket timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = SOCKET_TIMEOUT_US;
    int success = setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
    if (success < 0) {
        cout << "Error setting the timeout to the CAN socket" << endl;
        exit(1);
    }

    // Bind the socket to the CAN bus
    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_bus);
    ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));  // Init
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int result = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (result < 0) {
        cout << "Binding error! Exiting" << endl;
        exit(1);
    }
    else
        cout << "Socket successfully bound to the CAN bus " << endl;

    return s;
}


void enterMITMode(int id)
{
    struct can_frame frame;
    frame.can_id = id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFC;

    // Send frame
    int nbytes = -1;
    timespec startWrite = KMR::CBM::time_s();
    nbytes = write(s, &frame, sizeof(can_frame));
    timespec endWrite = KMR::CBM::time_s();
    double written = KMR::CBM::get_delta_us(endWrite, startWrite);
    cout << endl << "Sent enable command in " << written << " us" << endl;

    if (nbytes < 0)
        cout << "Error sending MIT mode request" << endl;

    nbytes = -1;
    // Receive frame
    timespec startConfirm = KMR::CBM::time_s();
    while(nbytes < 0) {
        timespec startRead = KMR::CBM::time_s();
        nbytes = read(s, &frame, sizeof(can_frame));
        timespec endRead = KMR::CBM::time_s();
        double read = KMR::CBM::get_delta_us(endRead, startRead);
        //cout << "Read function returned after " << read << " us" << endl;

        if (nbytes == 0)
            cout << "Read bytes = 0" << endl;
        //else if (nbytes < 0)
        //    cout << "No packet" << endl;
        else if (nbytes > 0) {
            //cout << endl;
            //cout << "CAN id: 0x" << convertToHex(frame.can_id) << endl;
            //cout << endl;
            //cout << "Data 0: 0x" << convertToHex( frame.data[0]) << endl;
            //cout << "Data 1: 0x" << convertToHex( frame.data[1]) << endl;
            //cout << "Data 2: 0x" << convertToHex( frame.data[2]) << endl;
            //cout << "Data 3: 0x" << convertToHex( frame.data[3]) << endl;
            //cout << "Data 4: 0x" << convertToHex( frame.data[4]) << endl;
            //cout << "Data 5: 0x" << convertToHex( frame.data[5]) << endl;
            //cout << "Data 6: 0x" << convertToHex( frame.data[6]) << endl;
            //cout << "Data 7: 0x" << convertToHex( frame.data[7]) << endl;

            cout << "Motor enabled" << endl;
        }
    }

    timespec endConfirm = KMR::CBM::time_s();
    double confirm = KMR::CBM::get_delta_us(endConfirm, startConfirm);
    cout << "Motor enable confirmation received in " << confirm << " us" << endl;
}


void exitMITMode(int id)
{
    struct can_frame frame;
    frame.can_id = id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFD;

    // Send frame
    int nbytes = -1;
    timespec startWrite = KMR::CBM::time_s();
    nbytes = write(s, &frame, sizeof(can_frame));
    timespec endWrite = KMR::CBM::time_s();
    double written = KMR::CBM::get_delta_us(endWrite, startWrite);
    cout << endl << "Sent disable command in " << written << " us" << endl;

    if (nbytes < 0)
        cout << "Error entering MIT mode" << endl;

    nbytes = -1;
    // Receive frame
    timespec startConfirm = KMR::CBM::time_s();
    while(nbytes < 0) {

        timespec startRead = KMR::CBM::time_s();
        nbytes = read(s, &frame, sizeof(can_frame));
        timespec endRead = KMR::CBM::time_s();
        double read = KMR::CBM::get_delta_us(endRead, startRead);
        //cout << "Read function returned after " << read << " us" << endl;

        if (nbytes == 0)
            cout << "Read bytes = 0" << endl;

        if (nbytes > 0) {
            //cout << endl;
            //cout << "CAN id: 0x" << convertToHex(frame.can_id) << endl;
            //cout << endl;
            //cout << "Data 0: 0x" << convertToHex( frame.data[0]) << endl;
            //cout << "Data 1: 0x" << convertToHex( frame.data[1]) << endl;
            //cout << "Data 2: 0x" << convertToHex( frame.data[2]) << endl;
            //cout << "Data 3: 0x" << convertToHex( frame.data[3]) << endl;
            //cout << "Data 4: 0x" << convertToHex( frame.data[4]) << endl;
            //cout << "Data 5: 0x" << convertToHex( frame.data[5]) << endl;
            //cout << "Data 6: 0x" << convertToHex( frame.data[6]) << endl;
            //cout << "Data 7: 0x" << convertToHex( frame.data[7]) << endl;

            cout << "Motor disabled" << endl;
        }
    }
    timespec endConfirm = KMR::CBM::time_s();
    double confirm = KMR::CBM::get_delta_us(endConfirm, startConfirm);
    cout << "Disable command confirmation received in " << confirm << " us" << endl;
}

void setZeroPosition(int id)
{
    struct can_frame frame;
    frame.can_id = id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0xFF;
    frame.data[1] = 0xFF;
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0xFF;
    frame.data[7] = 0xFE;

    // Send frame
    int nbytes = -1;
    timespec startWrite = KMR::CBM::time_s();
    nbytes = write(s, &frame, sizeof(can_frame));
    timespec endWrite = KMR::CBM::time_s();
    double written = KMR::CBM::get_delta_us(endWrite, startWrite);
    cout << endl << "Sent zero position in " << written << " us" << endl;
    if (nbytes < 0)
        cout << "Error sending zero position" << endl;

    nbytes = -1;
    // Receive frame
    timespec startConfirm = KMR::CBM::time_s();
    while(nbytes < 0) {
        nbytes = read(s, &frame, sizeof(struct can_frame));

        if (nbytes == 0)
            cout << "Read bytes = 0" << endl;

        if (nbytes > 0) {
            //cout << endl;
            //cout << "CAN id: 0x" << convertToHex(frame.can_id) << endl;
            //cout << endl;
            //cout << "Data 0: 0x" << convertToHex( frame.data[0]) << endl;
            //cout << "Data 1: 0x" << convertToHex( frame.data[1]) << endl;
            //cout << "Data 2: 0x" << convertToHex( frame.data[2]) << endl;
            //cout << "Data 3: 0x" << convertToHex( frame.data[3]) << endl;
            //cout << "Data 4: 0x" << convertToHex( frame.data[4]) << endl;
            //cout << "Data 5: 0x" << convertToHex( frame.data[5]) << endl;
            //cout << "Data 6: 0x" << convertToHex( frame.data[6]) << endl;
            //cout << "Data 7: 0x" << convertToHex( frame.data[7]) << endl;

            cout << "Zero position set" << endl;
        }
    }
    timespec endConfirm = KMR::CBM::time_s();
    double confirm = KMR::CBM::get_delta_us(endConfirm, startConfirm);
    cout << "Zero position confirm received in " << confirm << " us" << endl;
}

void stopMotor(int id)
{
    struct can_frame frame;
    frame.can_id = id;
    frame.len = FRAME_LENGTH;
    frame.data[0] = 0x7F;
    frame.data[1] = 0xFF;
    frame.data[2] = 0x7F;
    frame.data[3] = 0xF0;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x07;
    frame.data[7] = 0xFF;

    // Send frame
    int nbytes = -1;
    timespec startWrite = KMR::CBM::time_s();
    nbytes = write(s, &frame, sizeof(can_frame));
    timespec endWrite = KMR::CBM::time_s();
    double written = KMR::CBM::get_delta_us(endWrite, startWrite);

    if (nbytes < 0)
        cout << "Error sending stop command" << endl;

    nbytes = -1;
    // Receive frame
    timespec startConfirm = KMR::CBM::time_s();
    while(nbytes < 0) {
        nbytes = read(s, &frame, sizeof(can_frame));

        if (nbytes == 0)
            cout << "Read bytes = 0" << endl;
        else if (nbytes > 0) {
            //cout << endl;
            //cout << "CAN id: 0x" << convertToHex(frame.can_id) << endl;
            //cout << endl;
            //cout << "Data 0: 0x" << convertToHex( frame.data[0]) << endl;
            //cout << "Data 1: 0x" << convertToHex( frame.data[1]) << endl;
            //cout << "Data 2: 0x" << convertToHex( frame.data[2]) << endl;
            //cout << "Data 3: 0x" << convertToHex( frame.data[3]) << endl;
            //cout << "Data 4: 0x" << convertToHex( frame.data[4]) << endl;
            //cout << "Data 5: 0x" << convertToHex( frame.data[5]) << endl;
            //cout << "Data 6: 0x" << convertToHex( frame.data[6]) << endl;
            //cout << "Data 7: 0x" << convertToHex( frame.data[7]) << endl;

            //cout << "Entered MIT mode" << endl;
        }
    }

    struct timeval tv;
    ioctl (s, SIOCGSTAMP, &tv);
    timespec endConfirmCAN = KMR::CBM::convert_to_timespec(tv);
    timespec endConfirm = KMR::CBM::time_s();
    double confirm = KMR::CBM::get_delta_us(endConfirm, startConfirm);
    double confirmCAN = KMR::CBM::get_delta_us(endConfirmCAN, startConfirm);

    if (confirm > 1000 || confirmCAN > 1000) {
        cout << "Stopping received in " << confirm << " us" << endl;
        cout << "CAN stopping confirm received in " << confirmCAN << " us" << endl;
    }

}