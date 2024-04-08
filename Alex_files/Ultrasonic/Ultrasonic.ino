#define SPEED_OF_SOUND 340 // Speed of sound in ms-1 
#define ULTRASONIC 12 // define ultrasonic sensor to be Pin 12


double readUltrasonic(){ // detect distance of ultrasonic sensor from any objects in front of it
  pinMode(ULTRASONIC, OUTPUT); // set ultrasonic pin to output mode
  digitalWrite(ULTRASONIC, HIGH); // emit pulse from ultasonic sensor
  delayMicroseconds(10); // delay 10 microseconds
  digitalWrite(ULTRASONIC, LOW); // stop emitting sound from ultrasonic sensor
  pinMode(ULTRASONIC, INPUT); // set ultrasonic pin to input mode
  double duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT); // measure time taken to detect echo from initial ultrasonic pulse
  double dist = duration / 2 / 1000000 * SPEED_OF_SOUND * 100; // calculate distance of object from ultrasonic sensor in cm
  Serial.print("Distance by ultrasonic: ");
  Serial.println(dist); // print distance of object from ultrasonic sensor in cm to serial monitor
  return dist; // return distance of object from ultrasonic sensor in cm
}

void setup() {
  pinMode(ULTRASONIC, OUTPUT);
  digitalWrite(ULTRASONIC, LOW);
  Serial.begin(9600);
}

void loop(){
  double distance = readUltrasonic();
  Serial.println(distance);
}