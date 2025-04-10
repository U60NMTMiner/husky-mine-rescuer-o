#include <SPI.h>
#include <mcp2515.h>
#include "frames.hpp"

// TODO fix pin numbers
#define CAN_PIN 53

#define STEP_VERT_PIN 10
#define DIR_VERT_PIN 3
#define INLIM_VERT_PIN 4
#define OUTLIM_VERT_PIN 5

#define STEP_PUSH_PIN 6
#define DIR_PUSH_PIN 7
#define INLIM_PUSH_PIN 8
#define OUTLIM_PUSH_PIN 9

struct can_frame canMsg;
MCP2515 mcp2515(CAN_PIN);

int speed = 255; // CANNOT BE 0

bool estop = false;
bool inlim_vert = false;
bool outlim_vert = false;
bool inlim_push = false;
bool outlim_push = false;

/*
0 - waiting
1 - down
2 - extend
3 - retract
4 - up
*/
int state = 0;
const int STATE_MAX = 4; // Number of states

// Motor: false - vert, true - push
// Dir: false - backward, true - forward
void step(bool motor, bool dir); 

void setup()
{
    while (!Serial);
    Serial.begin(9600);
    SPI.begin();               //Begins SPI communication

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
    mcp2515.setNormalMode();

    pinMode(STEP_VERT_PIN, OUTPUT);
    pinMode(DIR_VERT_PIN, OUTPUT);
    pinMode(INLIM_VERT_PIN, INPUT);
    pinMode(OUTLIM_VERT_PIN, INPUT);

    pinMode(STEP_PUSH_PIN, OUTPUT);
    pinMode(DIR_PUSH_PIN, OUTPUT);
    pinMode(INLIM_PUSH_PIN, INPUT);
    pinMode(OUTLIM_PUSH_PIN, INPUT);
}

void loop()
{
    // Receive CAN
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
    {
        // Set estop or drop node if not estopped or currently
        // dropping node 
        if (canMsg.can_id == int(frames::ESTOP))
            estop = canMsg.data[0];
        else if (canMsg.can_id == int(frames::NODE) && !estop && state == 0)
            state = 1;
    }

    // Don't do anything if estopped
    if (estop) { return; }

    // Check limit switches
    outlim_vert = !digitalRead(OUTLIM_VERT_PIN);
    inlim_vert = !digitalRead(INLIM_VERT_PIN);
    outlim_push = !digitalRead(OUTLIM_PUSH_PIN);
    inlim_push = !digitalRead(INLIM_PUSH_PIN);

    switch (state)
    {
    case 1: // Moving down
        if (!outlim_vert)
            step(false, true);
        else
            state++;
        break;
    case 2: // Push node out
        if (!outlim_push)
            step(true, true);
        else
            state++;
        break;
    case 3: // Retract back in
        if (!inlim_push)
            step(true, false);
        else
            state++;
        break;
    case 4: // Move up
        if (!inlim_vert)
            step(false, false);
        else
            state++;
        break;
    default: // Do nothing
        break;
    }

    // Reset state
    if (state > STATE_MAX) { state = 0; }
}

void step(bool motor, bool dir)
{
    // Select proper pins based on motor
    int step_p;
    int dir_p;
    if (motor) {
        step_p = STEP_PUSH_PIN;
        dir_p = DIR_PUSH_PIN;
    } else {
        step_p = STEP_VERT_PIN;
        dir_p = DIR_VERT_PIN;
    }

    // Calculate delay based on speed
    if (speed < 1) { speed = 1; }
    int delay = 255*5/speed;
    
    // Step once
    digitalWrite(dir_p, dir);
    digitalWrite(step_p, HIGH);
    delayMicroseconds(delay);
    digitalWrite(step_p, LOW);
    delayMicroseconds(delay);
}