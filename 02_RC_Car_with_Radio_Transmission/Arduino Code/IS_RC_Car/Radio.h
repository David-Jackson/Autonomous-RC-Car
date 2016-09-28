#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"


#include "Accelerometer.h"


const int serialBufferSize = 200;     // buffer size for input
char  serialBuffer[serialBufferSize]; // buffer for input
const int serialMaxArgs = 8;          // max CSV message args 
char* serialArgs[serialMaxArgs];      // args pointers
int   inputTiming = 1;                // serial reading timing in ms

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

const int max_payload_size = 32;

char receive_payload[max_payload_size+1]; // +1 to allow room for a terminating NULL char


void transmit(String data) {
  String payload_string = data;

  Serial.println(payload_string);

  String space = " ";
  static char send_payload[50];
  for (int i = 0; i < payload_string.length(); i++) {
    send_payload[i] = payload_string.charAt(i);
  }
  for (int i = payload_string.length(); i < 51;  i++) {
    send_payload[i] = space.charAt(0);
  }

  //radio.write( send_payload, max_payload_size );
}

void Radio_Setup() {
  // Setup and configure rf radio
  radio.begin();
  // Open pipes to other nodes for communication
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  radio.stopListening();
  // Dump the configuration of the rf unit for debugging
  radio.printDetails();
  // rx buffer clearing
  while (Serial.available() > 0) {
    byte c = Serial.read();
  }
}

void Radio_Loop() {
  transmit("TIME:"+(String)millis());
  transmit(getAccel());
}

