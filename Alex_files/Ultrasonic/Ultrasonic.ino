#define SPEED_OF_SOUND 340 // Speed of sound in ms-1 
#define ULTRASONIC 12 // define ultrasonic sensor to be Pin 12
#define TIMEOUT 4000


double readUltrasonic(){ // detect distance of ultrasonic sensor from any objects in front of it
  pinMode(ULTRASONIC, OUTPUT); // set ultrasonic pin to output mode
  digitalWrite(ULTRASONIC, HIGH); // emit pulse from ultasonic sensor
  delayMicroseconds(10); // delay 10 microseconds
  digitalWrite(ULTRASONIC, LOW); // stop emitting sound from ultrasonic sensor
  pinMode(ULTRASONIC, INPUT); // set ultrasonic pin to input mode
  double duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT); // measure time taken to detect echo from initial ultrasonic pulse
  double dist = duration / 2 / 1000000 * SPEED_OF_SOUND * 100; // calculate distance of object from ultrasonic sensor in cm
  return dist; // return distance of object from ultrasonic sensor in cm
}

void setup() {
  pinMode(ULTRASONIC, OUTPUT);
  digitalWrite(ULTRASONIC, LOW);
  Serial.begin(9600);
}

void sendDist(uint32_t distance) {
  TPacket distancePacket;
  distancePacket.packetType = PACKET_TYPE_RESPONSE;
  distancePacket.command = RESP_DISTANCE;
  distancePacket.params[0] = distance;
  sendResponse(&distancePacket);
  sendOK();
}

void loop(){
  double distance = readUltrasonic();
  sendDist((uint32_t)distance);
 if(distance >= 2 && distance <= 10) {
    readColour();
  }
}
