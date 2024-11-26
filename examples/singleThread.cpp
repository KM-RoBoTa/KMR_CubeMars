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

#define FRAME_LENGTH 8

struct sockaddr_can addr;
struct ifreq ifr;
int s;


// Fcts
int openSocket();
void enterMITMode(int id);
void exitMITMode(int id);
void setZeroPosition(int id);


using namespace std;


int main()
{
    s = openSocket();

    enterMITMode(1);

    sleep(3);

    setZeroPosition(1);

    sleep(3);

    exitMITMode(1);

    sleep(3);

    // Close the socket
    close(s);
}


int openSocket()
{
    // Open a socket to be able to communicate over a CAN network
    int s = socket(PF_CAN, SOCK_RAW|SOCK_NONBLOCK, CAN_RAW);
    if (s < 0) {
        cout << "Error opening the socket! Exiting" << endl;
        exit(1);
    }
    else   
        cout << "Socket opened successfully" << endl;

    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
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
        cout << "Binding ok " << endl;

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
    timespec startWrite = time_s();
    nbytes = write(s, &frame, sizeof(can_frame));
    timespec endWrite = time_s();
    double written = get_delta_us(endWrite, startWrite);
    cout << endl << "Sent MIT enter in " << written << " us" << endl;

    if (nbytes < 0)
        cout << "Error entering MIT mode" << endl;

    nbytes = -1;
    // Receive frame
    timespec startConfirm = time_s();
    while(nbytes < 0) {
        nbytes = read(s, &frame, sizeof(can_frame));

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

            cout << "Entered MIT mode" << endl;
        }
    }
    timespec endConfirm = time_s();
    double confirm = get_delta_us(endConfirm, startConfirm);
    cout << "MIT entrance confirm received in " << confirm << " us" << endl;
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
    timespec startWrite = time_s();
    nbytes = write(s, &frame, sizeof(can_frame));
    timespec endWrite = time_s();
    double written = get_delta_us(endWrite, startWrite);
    cout << endl << "Sent MIT exit in " << written << " us" << endl;

    if (nbytes < 0)
        cout << "Error entering MIT mode" << endl;

    nbytes = -1;
    // Receive frame
    timespec startConfirm = time_s();
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

            cout << "Exited MIT mode" << endl;
        }
    }
    timespec endConfirm = time_s();
    double confirm = get_delta_us(endConfirm, startConfirm);
    cout << "MIT exit confirm received in " << confirm << " us" << endl;
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
    timespec startWrite = time_s();
    nbytes = write(s, &frame, sizeof(can_frame));
    timespec endWrite = time_s();
    double written = get_delta_us(endWrite, startWrite);
    cout << endl << "Sent zero position in " << written << " us" << endl;
    if (nbytes < 0)
        cout << "Error sending zero position" << endl;

    nbytes = -1;
    // Receive frame
    timespec startConfirm = time_s();
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
    timespec endConfirm = time_s();
    double confirm = get_delta_us(endConfirm, startConfirm);
    cout << "Zero position confirm received in " << confirm << " us" << endl;
}