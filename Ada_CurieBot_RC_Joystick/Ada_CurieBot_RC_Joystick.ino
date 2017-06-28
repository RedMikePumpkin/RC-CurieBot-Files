/*********************************************************************
 This is CurieBot RC -- the Arduino 101 based robot using Bluetooth LE
 Modified to drive a 3-wheeled BLE Robot Rover! by http://james.devi.to

 Pick one up today in the Adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

  #include <CurieBLE.h>
  #include <BLEPeripheral.h>
  #include "BLESerial.h"

  BLESerial ble = BLESerial();

static int lightPin = 13;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// And connect 2 DC motors to port M1 & M2 !
Adafruit_DCMotor *L_MOTOR = AFMS.getMotor(1);
Adafruit_DCMotor *R_MOTOR = AFMS.getMotor(2);

float leftMotorSpeed = 0.0;
float rightMotorSpeed = 0.0;
bool lightState = 0;

#define BLUETOOTH_NAME                 "CurieBot" //Name your RC here
#define BLE_READPACKET_TIMEOUT         500   // Timeout in ms waiting to read a response

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Stream *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

char buf[60];

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  //while (!Serial);// For use only while plugged into USB an with Serial Monitor open
  AFMS.begin();  // create with the default frequency 1.6KHz

  // turn off both motors
  L_MOTOR->setSpeed(0);
  R_MOTOR->setSpeed(0);
  L_MOTOR->run(RELEASE);
  R_MOTOR->run(RELEASE);
    
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Robot Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialize the module */
  BLEsetup();

  pinMode(lightPin, OUTPUT);

}

float x, y;

unsigned long lastAccelPacket = 0;

void loop(void)
{
  // read new packet data
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  Serial.print("Packet type: ");
  Serial.println(packetbuffer[1]);

  // Read from Accelerometer input
  if( accelMode() ) {
    lastAccelPacket = millis();
    return;
  }

  // Stop motors if accelerometer data is turned off (100ms timeout)
  if((millis() - lastAccelPacket) > 100) {
    L_MOTOR->run(RELEASE);
    R_MOTOR->run(RELEASE);
    leftMotorSpeed = 0.0;
    rightMotorSpeed = 0.0;
    lightState = 0;
    digitalWrite(lightPin, lightState);
    return;
  }

  // process buttons
  buttonMode();
}

bool accelMode(){
  if (packetbuffer[1] == 'A') {
    memcpy(&x, packetbuffer + 2, sizeof x);
    memcpy(&y, packetbuffer + 6, sizeof y);
    Serial.print("x = ");
    Serial.print(x);
    Serial.print("; y = ");
    Serial.println(y);

    float leftMotorOutput = 0;
    float rightMotorOutput = 0;

    // Differential steering code adaptapted from
    //http://www.impulseadventure.com/elec/robot-differential-steering.html 
    
    // Calculate Drive Turn output due to Joystick X input
    if (y >= 0) {
      // Forward
      if (x >= 0) {
        // Turn right - slow down right motor
        leftMotorOutput = 1.0;
        rightMotorOutput = 1.0 - x;
      } else {
        // Turn left - slow down left motor
        leftMotorOutput = 1.0 + x;
        rightMotorOutput = 1.0;
      }
    } else {
      // Reverse
      if (x >= 0) {
        // Turn left - slow down left motor
        leftMotorOutput = 1.0 - x;
        rightMotorOutput = 1.0;
      } else {
        // Turn right - slow down right motor
        leftMotorOutput = 1.0;
        rightMotorOutput = 1.0 + x;    
      }
    }

    // Scale Drive output due to Joystick Y input (throttle)
    leftMotorOutput *= y;
    rightMotorOutput *= y;

    // Now calculate pivot amount
    // - Strength of pivot based on Joystick X input
    // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
    float fPivLimit = 0.1;
    float fPivScale = (abs(y)>fPivLimit)? 0.0 : (1.0 - abs(y)/fPivLimit);

    // Calculate final mix of Drive and Pivot
    leftMotorOutput = (1.0-fPivScale)*leftMotorOutput + fPivScale*x;
    leftMotorOutput = (1.0-fPivScale)*leftMotorOutput + fPivScale*x;


    Serial.print("leftMotorOutput = ");
    Serial.print(leftMotorOutput);
    Serial.print("; rightMotorOutput = ");
    Serial.println(rightMotorOutput);

    // limit acceleration
    if (leftMotorOutput - leftMotorSpeed > 0.1) {
      leftMotorOutput = leftMotorSpeed + 0.1;
    } else if (leftMotorSpeed - leftMotorOutput > 0.1) {
      leftMotorOutput = leftMotorSpeed - 0.1;
    }
    leftMotorSpeed = leftMotorOutput;

    if (rightMotorOutput - rightMotorSpeed > 0.1) {
      rightMotorOutput = rightMotorSpeed + 0.1;
    } else if (rightMotorSpeed - rightMotorOutput > 0.1) {
      rightMotorOutput = rightMotorSpeed - 0.1;
    }
    rightMotorSpeed = rightMotorOutput;

    // Convert to Motor Speed to PWM range, and configure motors
    if (leftMotorSpeed > 0) {
      L_MOTOR->run( FORWARD );
      L_MOTOR->setSpeed(leftMotorSpeed * 255); 
    } else if (leftMotorSpeed < 0) {
      L_MOTOR->run( BACKWARD );
      L_MOTOR->setSpeed(-leftMotorSpeed * 255); 
    } else {
      L_MOTOR->run( RELEASE );
      L_MOTOR->setSpeed(0);
    }

    if (rightMotorSpeed > 0) {
      R_MOTOR->run( FORWARD );
      R_MOTOR->setSpeed(rightMotorSpeed * 255); 
    } else if (rightMotorSpeed < 0) {
      R_MOTOR->run( BACKWARD );
      R_MOTOR->setSpeed(-rightMotorSpeed * 255); 
    } else {
      R_MOTOR->run( RELEASE );
      R_MOTOR->setSpeed(0);
    }
    return true;
  }
  return false;
}

bool isMoving = false;
unsigned long lastPress = 0;

bool buttonMode(){

   // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';

    Serial.print("Button #"); Serial.print(buttnum);
    if (pressed) 
      Serial.println(" pressed");
    else
      Serial.println(" released");
    
    if (pressed) {
      if(buttnum == 1) {
        // toggle the light
        lightState = !lightState;
      }
    }
    digitalWrite(lightPin, lightState);
    return true; 
  }

  return false;

}

void BLEsetup() {
  // using Curie
  ble.setLocalName("CurieBot");
  ble.begin();
}
