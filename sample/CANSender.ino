// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <CAN.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CAN Sender");

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

int engine_rev = 0; //回転数
int car_speed = 0;

void loop() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending packet ... ");

  CAN.beginPacket(0x3d9);
  CAN.write(0);
  CAN.write(0);
  CAN.write(engine_rev *4 / 256);  // [km/h] と合わせるためにｘ４
  CAN.write(engine_rev *4 % 256);
  CAN.write(0);
  CAN.write(0);
  CAN.write(0);
  CAN.write(0);
  CAN.endPacket();

  CAN.beginPacket(0x3e9);
  CAN.write(car_speed *128 / 256);  // [rpm]に合わせるためにｘ１２８
  CAN.write(car_speed *128 % 256);
  CAN.write(0);
  CAN.write(0);
  CAN.write(0);
  CAN.write(0);
  CAN.write(0);
  CAN.write(0);
  CAN.endPacket();

  Serial.println("done");

  engine_rev += 100;
  engine_rev %= 7000;
  if(engine_rev < 2000)
    engine_rev = 2000;
  
  car_speed += 2;
  car_speed %= 240;
  delay(30);


  Serial.println("done");
  delay(30);
}

