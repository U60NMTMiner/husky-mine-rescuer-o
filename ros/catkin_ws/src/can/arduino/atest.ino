#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;

MCP2515 mcp2515(10);

void setup()
{
    while (!Serial);
    Serial.begin(9600);
    SPI.begin();               //Begins SPI communication

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
    mcp2515.setNormalMode();
}

void loop()
{
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) // To receive data (Poll Read)
    {
        for (int i = 0; i < canMsg.can_dlc; i++)
        {
            Serial.print(char(canMsg.data[i]));
            Serial.print(" ");
        }
        Serial.println();
    }
}