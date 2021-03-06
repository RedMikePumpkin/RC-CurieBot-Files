// Copyright (c) Sandeep Mistry. All rights reserved.
// from https://github.com/sandeepmistry/arduino-BLEPeripheral/blob/master/examples/serial/
// Licensed under the MIT license. See LICENSE file in the project root for full license information.


#ifndef _BLE_SERIAL_H_
#define _BLE_SERIAL_H_

#include <Arduino.h>
#if defined(_VARIANT_ARDUINO_101_X_)
#include <CurieBLE.h>
#define _MAX_ATTR_DATA_LEN_ BLE_MAX_ATTR_DATA_LEN
#else
#include <BLEPeripheral.h>
#define _MAX_ATTR_DATA_LEN_ BLE_ATTRIBUTE_MAX_VALUE_LENGTH
#endif

class BLESerial : public BLEPeripheral, public Stream
{
  public:
    BLESerial(unsigned char req = 0, unsigned char rdy = 0, unsigned char rst = 0);

    void begin(...);
    void poll();
    void end();

    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);
    virtual size_t write(uint8_t byte);
    using Print::write;
    virtual operator bool();

  private:
    unsigned long _flushed;
    static BLESerial* _instance;

    size_t _rxHead;
    size_t _rxTail;
    size_t _rxCount() const;
    uint8_t _rxBuffer[_MAX_ATTR_DATA_LEN_];
    size_t _txCount;
    uint8_t _txBuffer[_MAX_ATTR_DATA_LEN_];

    BLEService _uartService = BLEService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    BLEDescriptor _uartNameDescriptor = BLEDescriptor("2901", "UART");
    BLECharacteristic _rxCharacteristic = BLECharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWriteWithoutResponse, _MAX_ATTR_DATA_LEN_);
    BLEDescriptor _rxNameDescriptor = BLEDescriptor("2901", "RX - Receive Data (Write)");
    BLECharacteristic _txCharacteristic = BLECharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, _MAX_ATTR_DATA_LEN_);
    BLEDescriptor _txNameDescriptor = BLEDescriptor("2901", "TX - Transfer Data (Notify)");

    void _received(const uint8_t* data, size_t size);
    static void _received(BLECentral& /*central*/, BLECharacteristic& rxCharacteristic);
};

#endif

