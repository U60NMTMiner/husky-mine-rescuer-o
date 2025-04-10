enum class frames {
    ESTOP=0x000,
    DRAWER=0x001,
    NODE=0x002,
};

#include <SPI.h>
#include <mcp2515.h>

#define CAN_PIN 53
#define STEP_PIN 10
#define DIR_PIN 3
#define INLIM_PIN 4
#define OUTLIM_PIN 5
#define ENABLE_PIN 12 // Must hold HIGH to disable

struct can_frame canMsg;
MCP2515 mcp2515(CAN_PIN);
int speed = 255; // CANNOT BE 0
bool estop = false;
bool extend = true;  // false = up, true = down
bool outlim = false;
bool inlim = false;

void step(bool dir);

void setup()
{
    while (!Serial);
    Serial.begin(1000000);
    Serial.println("This program is a workaround,");
    Serial.println("it only moves up or down.");
    SPI.begin();               //Begins SPI communication

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
    mcp2515.setNormalMode();

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(INLIM_PIN, INPUT);
    pinMode(OUTLIM_PIN, INPUT);
    pinMode(ENABLE_PIN, OUTPUT);
}

void loop()
{  
    // Receive CAN
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
    {
      Serial.println("Something came in");
        if (canMsg.can_id == int(frames::ESTOP)){
            estop = canMsg.data[0];
            Serial.println("Estop");
            }
        else if (canMsg.can_id == int(frames::NODE) && !estop){
            extend = canMsg.data[0];
            Serial.println("CAN Recieved");
        }
    }
    

    // Don't do anything if estopped
    if (estop) { return; }

    // Check limit switches
    outlim = !digitalRead(OUTLIM_PIN);
    inlim = !digitalRead(INLIM_PIN);

    //Serial.print("outlim: "); Serial.print(outlim);
    //Serial.print(" inlim: "); Serial.println(inlim);

    if (!inlim && !extend) {
      step(true);
    } else if (!outlim && extend) {
      step(false);
    } else if (inlim || outlim) {
      digitalWrite(ENABLE_PIN, HIGH);
    }
}

void step(bool dir)
{
  
    if (speed < 1) { speed = 1; }
    int delay = 255*5/speed;
    //Serial.print("Step. Dir = ");
    //Serial.println(dir);
    
    digitalWrite(DIR_PIN, dir);
    digitalWrite(ENABLE_PIN, LOW);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delay);

    Serial.print("Step. Dir = ");
    Serial.println(dir);
}