#include <ESP32-TWAI-CAN.hpp>

// Simple sketch that querries OBD2 over CAN for coolant temperature
// Showcasing simple use of ESP32-TWAI-CAN library driver.

// Default for ESP32
#define CAN_TX 5
#define CAN_RX 4


CanFrame rxFrame;

void sendObdFrame(uint8_t obdId) {
  CanFrame obdFrame = { 0 };
  obdFrame.identifier = 0x7DF;  // Default OBD2 address;
  obdFrame.extd = 0;
  obdFrame.data_length_code = 8;
  obdFrame.data[0] = 2;
  obdFrame.data[1] = 1;
  obdFrame.data[2] = obdId;
  obdFrame.data[3] = 0xAA;  // Best to use 0xAA (0b10101010) instead of 0
  obdFrame.data[4] = 0xAA;  // CAN works better this way as it needs
  obdFrame.data[5] = 0xAA;  // to avoid bit-stuffing
  obdFrame.data[6] = 0xAA;
  obdFrame.data[7] = 0xAA;
  // Accepts both pointers and references
  ESP32Can.writeFrame(obdFrame);  // timeout defaults to 1 ms
}

void setup() {
  // Setup serial for debbuging.
  Serial.begin(115200);

  // Set pins
  ESP32Can.setPins(CAN_TX, CAN_RX);

  // You can set custom size for the queues - those are default
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);

  // .setSpeed() and .begin() functions require to use TwaiSpeed enum,
  // but you can easily convert it from numerical value using .convertSpeed()
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));

  // You can also just use .begin()..
  if (ESP32Can.begin()) {
    Serial.println("CAN bus started!");
  } else {
    Serial.println("CAN bus failed!");
  }

  // or override everything in one command;
  // It is also safe to use .begin() without .end() as it calls it internally
  if (ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10)) {
    Serial.println("CAN bus started!");
  } else {
    Serial.println("CAN bus failed!");
  }
}

int engine_rev = 0;  //回転数
int car_speed = 0;

void loop() {

  CanFrame frame = { 0 };
  frame.identifier = 0x3d9;  //  address;
  frame.extd = 0;
  frame.data_length_code = 8;
  frame.data[0] = 0;
  frame.data[1] = 0;
  frame.data[2] = (engine_rev * 4 / 256);
  frame.data[3] = (engine_rev * 4 % 256);  // engine revolution
  frame.data[4] = 0x0;                     //
  frame.data[5] = 0x0;                     //
  frame.data[6] = 0x0;
  frame.data[7] = 0x0;
  // Accepts both pointers and references
  ESP32Can.writeFrame(frame);  // timeout defaults to 1 ms

  delay(15);

  frame.identifier = 0x3e9;  //  address;
  frame.extd = 0;
  frame.data_length_code = 8;
  frame.data[0] = (car_speed * 128 / 256); //speed
  frame.data[1] = (car_speed * 128 / 256);
  frame.data[2] = 0x0;
  frame.data[3] = 0x0;  //
  frame.data[4] = 0x0;  //
  frame.data[5] = 0x0;  //
  frame.data[6] = 0x0;
  frame.data[7] = 0x0;
  // Accepts both pointers and references
  ESP32Can.writeFrame(frame);  // timeout defaults to 1 ms
  engine_rev += 20;
  engine_rev %= 7000;
  if (engine_rev < 1000)
    engine_rev = 1000;

  car_speed += 1;
  car_speed %= 330;
  delay(15);
/*
  // You can set custom timeout, default is 1000
  if (ESP32Can.readFrame(rxFrame, 10)) {
    // Comment out if too many frames
    Serial.printf("Received frame: %03X  \r\n", rxFrame.identifier);
    if (rxFrame.identifier == 0x7E8) {                                  // Standard OBD2 frame responce ID
      Serial.printf("Collant temp: %3d°C \r\n", rxFrame.data[3] - 40);  // Convert to °C
    }
  }
  */
}