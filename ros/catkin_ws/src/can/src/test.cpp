#include <iostream>
#include "../include/can/can.hpp"
#include <string>

static SocketCAN socket;

int main(int argc, char** argv)
{
    socket = SocketCAN("can0");
    socket.drawer(false);
}
