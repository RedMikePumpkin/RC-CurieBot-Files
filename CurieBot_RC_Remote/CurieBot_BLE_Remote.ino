/*
 * Copyright (c) 2017 Sergey Kiselev and Michael Kiselev.  All rights reserved.
 *
 */

/*
 * Sketch: CurieBot_BLE_Remote.ino
 *
 * Description:
 *  This sketch implements a remote control for CurieBot
 *  It uses joystick as an input device, and it sends the commands
 *  to CurieBot using Bluetooth Low Energy Central functionality,
 *  by writing these commands to Nordic UART TX characteristic
 *
 * Notes:
 *
 *  - Nordic BLE UART Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *  - Nordic BLE UART TX Characteristic: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 *
 */

#include <CurieBLE.h>

// variables for button
const int buttonPin = 2;
const int lightPin = 13;
const int joystickX = A0;
const int joystickY = A1;
int oldButtonState = LOW;


void setup() {
  Serial.begin(115200);

  // configure the button pin as input
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(joystickX, INPUT);
  pinMode(joystickY, INPUT);
  pinMode(lightPin, OUTPUT);

  // initialize the BLE hardware
  BLE.begin();

  Serial.println("BLE Central - CurieBot Control");

  // start scanning for peripherals
  BLE.scanForUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    // stop scanning
    BLE.stopScan();

    controlBot(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  }
}

void bluefruitSendFloat(BLECharacteristic tx_characteristic, char data_type, float values[])
{
  uint8_t packetbuffer[21];
  uint8_t i, sum = 0;
  
  packetbuffer[0] = '!';
  packetbuffer[1] = data_type;
  memcpy(packetbuffer + 2, &values[0], 4);
  memcpy(packetbuffer + 6, &values[1], 4);
  memcpy(packetbuffer + 10, &values[2], 4);

  for (i = 0; i < 14; i++) sum += packetbuffer[i];
  packetbuffer[14] = ~sum;
  
  tx_characteristic.writeValue(packetbuffer, 15);

}

void bluefruitSendButton(BLECharacteristic tx_characteristic, uint8_t button_number, uint8_t button_state)
{
  uint8_t packetbuffer[21];
  uint8_t sum;
  
  packetbuffer[0] = '!';
  sum = '!';
  packetbuffer[1] = 'B';
  sum += 'B';
  packetbuffer[2] = '0' + button_number;
  sum += '0' + button_number;
  packetbuffer[3] = '0' + button_state;
  sum += '0' + button_state;
  packetbuffer[4] = ~sum;

  tx_characteristic.writeValue(packetbuffer, 5);
}

void controlBot(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic txCharacteristic = peripheral.characteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

  if (!txCharacteristic) {
    Serial.println("Peripheral does not have UART TX characteristic!");
    peripheral.disconnect();
    return;
  } else if (!(txCharacteristic.properties() & BLEWriteWithoutResponse)) {
    Serial.println("Peripheral does not have a writable UART TX characteristic!");
    peripheral.disconnect();
    return;
  }

  digitalWrite(lightPin, HIGH);

  float curve = -4.0;
  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output 
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function


  while (peripheral.connected()) {
    // while the peripheral is connection
    float joystick_values[3];

    // read the button pin
    int buttonState = digitalRead(buttonPin);

    int x = analogRead(joystickX);
    int y = analogRead(joystickY);
    Serial.print("joystick X = ");
    Serial.print(x);
    Serial.print("; joystick Y = ");
    Serial.println(y);
    if (x < 512) {
      // negative value
      joystick_values[0] = -pow((511.0 - (float) x) / 512.0, curve);
    } else {
      // positive value
      joystick_values[0] = pow(((float) x - 512.0) / 512.0, curve);
    }
    if (y < 512) {
      // negative value
      joystick_values[1] = -pow((511.0 - (float) y) / 512.0, curve);
    } else {
      // positive value
      joystick_values[1] = pow(((float) y - 512.0) / 512.0, curve);
    }
    joystick_values[2] = 0;
    bluefruitSendFloat(txCharacteristic, 'A', joystick_values);

    if (oldButtonState != buttonState) {
      // button state changed
      if (buttonState == LOW) {
        bluefruitSendButton(txCharacteristic, 1, 1);
      } else {
        bluefruitSendButton(txCharacteristic, 1, 0);
      }
      oldButtonState = buttonState;
    }
    delay(100);
  }
  
  digitalWrite(lightPin, LOW);
  Serial.println("Peripheral disconnected");
}



