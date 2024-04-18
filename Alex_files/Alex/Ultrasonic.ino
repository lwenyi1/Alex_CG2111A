#include "constants.h"
#include "packet.h"
#include <math.h>

void setupUltrasonic() // Code for the ultrasonic sensor
{
  DDRA |= (1 << 4); // set trigger pin to output
  PORTA &= ~(1 << 4); // write LOW to trigger pin
  DDRA &= ~(1 << 5); // set echo pin to input
}

uint32_t readUltrasonic() { // detect distance of ultrasonic sensor from any objects in front of it
  PORTA |= (1 << 4); // emit pulse from ultasonic sensor
  delayMicroseconds(100); // delay 100 microseconds
  PORTA &= ~(1 << 4); // stop emitting sound from ultrasonic sensor
  double duration = pulseIn(ECHO, HIGH, TIMEOUT); // measure time taken to detect echo from initial ultrasonic pulse
  double dist = duration / 2 / 10000 * SPEED_OF_SOUND; // calculate distance of object from ultrasonic sensor
  return (uint32_t) round(dist); // return distance of object from ultrasonic sensor in cm
}

void sendDist(uint32_t distance) {
  TPacket distancePacket;
  distancePacket.packetType = PACKET_TYPE_RESPONSE;
  distancePacket.command = RESP_ULTRASONIC;
  distancePacket.params[0] = distance;
  sendResponse(&distancePacket);
}
