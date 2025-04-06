#include <can/can.hpp>
#include <can/frames.hpp>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <net/if.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace std;

fd::fd() : id(-1) {}
fd::fd(int n) : id(n) {}
fd::fd(fd &&other) {
    id = other.id;
    other.id = -1;
}
fd::~fd() {
    if (id != -1)
        close(id);
}

void fd::operator=(fd &&other) {
    id = other.id;
    other.id = -1;
}
int fd::operator*() { return id; }

int fd::into_raw() {
    int id_ = id;
    id = -1;
    return id_;
}

SocketCAN::SocketCAN() {}
SocketCAN::SocketCAN(const string &interface) {
    sock = socket(AF_CAN, SOCK_RAW, CAN_RAW);
    if (*sock == -1)
        throw string("Error creating socket");

    // Select CAN interface.
    ifreq req;
    strncpy(req.ifr_ifrn.ifrn_name, interface.data(),
            sizeof(req.ifr_ifrn.ifrn_name));
    if (ioctl(*sock, SIOCGIFINDEX, &req) == -1)
        throw string("Unable to select CAN interface");

    // Bind the socket to the network interface.
    sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = req.ifr_ifru.ifru_ivalue;
    if (bind(*sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == -1)
        throw string("Failed to bind socket to network interface");

    cout << "Successfully bound socket to interface." << endl;
}

void SocketCAN::transmit(const struct can_frame &cf) {
    if (write(*sock, &cf, sizeof(cf)) != sizeof(cf))
        throw string("Error transmitting CAN frame");
}

void SocketCAN::transmit(int can_id, uint8_t data[8]) {
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = 8 * sizeof(data[0]);
    memcpy(frame.data, data, frame.can_dlc);
    transmit(frame);
}

void SocketCAN::estop(bool estop)
{
    uint8_t msg[8];
    memset(msg, 0, sizeof(msg));
    msg[0] = estop;
    this->transmit(int(frames::ESTOP), msg);
}

void SocketCAN::drawer(bool extend)
{
    uint8_t msg[8];
    memset(msg, 0, sizeof(msg));
    msg[0] = extend;
    this->transmit(int(frames::DRAWER), msg);
}

void SocketCAN::drop_node()
{
    uint8_t msg[8];
    memset(msg, 0, sizeof(msg));
    this->transmit(int(frames::NODE), msg);
}

