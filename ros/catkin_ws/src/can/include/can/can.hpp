#ifndef CAN_H
#define CAN_H

#include <linux/can.h>
#include <string>

// Basic file descriptor wrapper. Not really intended for use outside
// SocketCAN.
class fd {
    int id;

  public:
    fd();
    fd(int n);
    fd(fd &&other);
    ~fd();

    void operator=(fd &&other);
    int operator*();

    int into_raw();
};

// CAN socket.
class SocketCAN {
    fd sock;
    void transmit(const struct can_frame &cf);
    void transmit(int can_id, uint8_t data[8]);

  public:
    SocketCAN();
    SocketCAN(const std::string &interface);
    
    void estop(bool estop);
    void drawer(bool drawer);
    void drop_node();
};

#endif
