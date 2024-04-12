#include <serialize.h>
#include <stdarg.h>
#include <math.h>

#include "packet.h"
#include "constants.h"

//Alex's length and breadth in cm
#define ALEX_LENGTH 25.7
#define ALEX_BREADTH 15.5
#define S0 22
#define S1 23
#define S2 24
#define S3 25
#define sensorOut A8
#define TRIG 26
#define ECHO 27
#define TIMEOUT 4000
#define SPEED_OF_SOUND 340

/*
   Alex's configuration constants
*/

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV      4

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.735

/*
      Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

//Counters for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftForwardRevs;
volatile unsigned long rightForwardRevs;
volatile unsigned long leftReverseRevs;
volatile unsigned long rightReverseRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//Variables to keep track of whether we've
//moved a commanded distance and keep track
//of our turning angle
unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

//Alex's diag and turning circumference
float alexDiagonal = 0.0;
float alexCirc = 0.0;

volatile TDirection dir;

unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long) ((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}


/*

   Alex Communication Routines.

*/

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);

}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.

  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...)
{
  //can be called the same way we call printf
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);

}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
   Setup and start codes for external interrupts and
   pullup resistors.

*/
// Enable pull up resistors on pins 18 and 19
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= ~0b00001100;
  PORTD |= 0b00001100;
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR()
{
  switch (dir)
  {
    case FORWARD: leftForwardTicks++;
      break;

    case BACKWARD: leftReverseTicks++;
      break;

    case LEFT: leftReverseTicksTurns++;
      break;

    case RIGHT: leftForwardTicksTurns++;
      break;
  }

  if (dir == FORWARD)
  {
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == BACKWARD)
  {
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
}

void rightISR()
{
  switch (dir)
  {
    case FORWARD: rightForwardTicks++;

    case BACKWARD: rightReverseTicks++;

    case LEFT: rightForwardTicksTurns++;

    case RIGHT: rightReverseTicksTurns++;
  }
}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
  DDRD = 0;
  EICRA = 0b10100000;
  EIMSK = 0b1100;
}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.

ISR(INT2_vect) {
  rightISR();
}

ISR(INT3_vect) {
  leftISR();
}

// Implement INT2 and INT3 ISRs above.

/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  PRR0 &= ~(1 << PRUSART0);
  UBRR0H = 0;
  UBRR0L = (unsigned char)103; // 103 for 9600baud
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Asynchronous USART Mode
  UCSR0A = 0; // Clear the bits of UCSR0A while setting up

  //Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.

  // Start the transmitter and receiver, but disable
  // all interrupts.
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{
  int count = 0;
  // Read data from the buffer
  while ((UCSR0A & (1 << RXC0))) {
    // Read the received byte and store it in the buffer
    buffer[count++] = UDR0;
  }

  return count;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  //    int count = 0;
  //    while (Serial.available())
  //    buffer[count++] = Serial.read();
  //
  //    return count; // returns the number of bytes read
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
    for (int i = 0; i < len; i++) {
      while (!(UCSR0A & (1 << UDRE0)));
      UDR0 = buffer[i]; // write each byte of the buffer
    }
  //Serial.write(buffer, len);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
   Alex's setup and run codes

*/

void setupUltrasonic() // Code for the ultrasonic sensor
{
  DDRA |= (1 << 4); // set trigger pin to output
  PORTA &= ~(1 << 4); // write LOW to trigger pin
  DDRA &= ~(1 << 5); // set echo pin to input
}

<<<<<<< HEAD
uint32_t readUltrasonic() { // detect distance of ultrasonic sensor from any objects in front of it
  PORTA |= (1 << 4); // emit pulse from ultasonic sensor
  delayMicroseconds(100); // delay 10 microseconds
=======
double readUltrasonic() { // detect distance of ultrasonic sensor from any objects in front of it
  PORTA |= (1 << 4); // emit pulse from ultasonic sensor
  delayMicroseconds(10); // delay 10 microseconds
>>>>>>> baa01359bf510395f9fc59aab5d7f560db217c98
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

float triangularMembership(int x, int a, int b, int c) {
  if (x < a) {
    return 0;
  }
  else if (a <= x && x <= b) {
    return (float)(x - a) / (b - a);
  }
  else if (b < x && x < c) {
    return (float)(c - x) / (c - b);
  }
  else {
    return 0;
  }
}

void evaluateColour(int r, int g, int b, TPacket *colour) {
  float redR = triangularMembership(r, 199, 369, 463);
  float greenR = triangularMembership(r, 400, 534, 584);
  float whiteR = triangularMembership(r, 160, 300, 412);    

  float redG = triangularMembership(g, 393, 518, 561);
  float greenG = triangularMembership(g, 337, 468, 528);
  float whiteG = triangularMembership(g, 153, 290, 398);  

  float redB = triangularMembership(b, 331, 369, 463);
  float greenB = triangularMembership(b, 400, 534, 584);
  float whiteB = triangularMembership(b, 134, 255, 346);   

  float weightedRed = ((r * redR) + (g * redG) + (b * redB))/(redR + redG + redB);
  float weightedGreen = ((r * greenR) + (g * greenG) + (b * greenB))/(greenR + greenG + greenB);
  float weightedWhite = ((r * whiteR) + (g * whiteG) + (b * whiteB))/(whiteR + whiteG + whiteB);

  if (weightedRed > weightedGreen && weightedRed > weightedWhite) {
    colour->params[3] = 0; //RED
  } else if (weightedGreen > weightedWhite) {
    colour->params[3] = 1; //GREEN
  } else {
    colour->params[3] = 2; //WHITE
  }
  return;
}

void readColour() {
  TPacket colour;
  colour.packetType = PACKET_TYPE_RESPONSE;
  colour.command = RESP_COLOUR;
  uint32_t frequency = 0;
  
  // Setting red filtered photodiodes to be read
  //PORTB &= ~((1 << 5)|(1 << 6)); // to be checked
   
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
   
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  colour.params[0] = frequency;

  delay(100);

  // Setting Green filtered photodiodes to be read
  //PORTB |= ((1 << 5)|(1 << 6)); // to be checked
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  colour.params[1] = frequency;

  delay(100);

  // Setting Blue filtered photodiodes to be read
  //PORTB &= ~(1 << 5);
  //PORTB |= (1 << 6); // to be checked
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  colour.params[2] = frequency;
  
  delay(100);

  evaluateColour(colour.params[0], colour.params[1], colour.params[2], &colour);
   
  sendResponse(&colour);
}

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  leftForwardRevs = 0;
  rightForwardRevs = 0;
  leftReverseRevs = 0;
  rightReverseRevs = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Alex's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_REVERSE:
      sendOK();
      backward((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_LEFT:
      sendOK();
      left((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_TURN_RIGHT:
      sendOK();
      right((double) command->params[0], (float) command->params[1]);
      break;

    case COMMAND_STOP:
      sendOK();
      stop();
      break;

    case COMMAND_GET_STATS:
      sendOK();
      sendStatus();
      break;

    case COMMAND_CLEAR_STATS:
      sendOK();
      clearOneCounter(command->params[0]);
      break;

    case COMMAND_COLOUR:
      sendOK();
      readColour();
      break;

    case COMMAND_ULTRASONIC:
      sendOK();
      uint32_t distance = readUltrasonic();
      sendDist(distance);
      break;

    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {


        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupUltrasonic();
  setupColour();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {
  // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

  // put your main code here, to run repeatedly:
   
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }

  if (deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      if (forwardDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD)
    {
      if (reverseDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == (TDirection) STOP)
    {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0)
  {
    if (dir == LEFT)
    {
      if (leftReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT)
    {
      if (rightReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == (TDirection)STOP)
    {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}
