#include "constants.h"
#include "packet.h"

void setupColour() {
  DDRA |= 0b00001111;
  pinMode(sensorOut, INPUT);

  // Setting frequency-scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  //PORTB = 0b00001000;
  //PORTB &= ~(1 << 4); // to be checked
  /** digitalWrite(S0,HIGH);// to baremetal
    digitalWrite(S1,LOW); */
}

void evaluateColour(int r, int g, int b, TPacket *colour) {
  float rgbArr[3][3] = {{0.6425624, 0.7596533, 1.1819191}, {1.1747332, 1.0639985, 0.9094113}, {0.9444405, 1.0347558, 1.095375}};
  float rgNew = (float)r / (float)g;
  float rbNew = (float)r / (float)b;
  float gbNew = (float)g / (float)b;
  float rgbCol[3] = {rgNew, rbNew, gbNew};

  float errCol[3] = {0, 0, 0};

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      errCol[i] += (rgbCol[j] - rgbArr[i][j]) * (rgbCol[j] - rgbArr[i][j]);
    }
  }

  float minimum = errCol[0];
  int min_index = 0;
  for (int k = 0; k < 3; k++) {
    if (errCol[k] < minimum) {
      minimum = errCol[k];
      min_index = k;
    }
  }
  colour->params[3] = min_index;
}

void readColour() {
  TPacket colour;
  colour.packetType = PACKET_TYPE_RESPONSE;
  colour.command = RESP_COLOUR;
  uint32_t frequency = 0;

  // Setting red filtered photodiodes to be read
  //PORTB &= ~((1 << 5)|(1 << 6)); // to be checked

  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  colour.params[0] = frequency;

  delay(100);

  // Setting Green filtered photodiodes to be read
  //PORTB |= ((1 << 5)|(1 << 6)); // to be checked
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  colour.params[1] = frequency;

  delay(100);

  // Setting Blue filtered photodiodes to be read
  //PORTB &= ~(1 << 5);
  //PORTB |= (1 << 6); // to be checked
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  colour.params[2] = frequency;

  delay(100);

  evaluateColour(colour.params[0], colour.params[1], colour.params[2], &colour);

  sendResponse(&colour);
}