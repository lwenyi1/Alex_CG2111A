// TCS230 or TCS3200 pins wiring to Arduino
#define S0 26
#define S1 25
#define S2 24
#define S3 23
#define sensorOut 22

// RGB LED pins (common anode)
#define RED_LED 9
#define GREEN_LED 10
#define BLUE_LED 11

// Stores frequency read by the photodiodes
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

void setup() {
  // Setting the outputs for sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  // Setting the sensorOut as an input
  pinMode(sensorOut, INPUT);
  
  // Setting frequency scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  
  // Setting the RGB LED pins as outputs
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  
  // Begin serial communication
  Serial.begin(9600);
}

void loop() {
  // Reading color frequencies
  readColorFrequency();
  
  // Logic to control the LED based on the detected color
  if (redFrequency > greenFrequency && redFrequency > blueFrequency) {
    // Red color detected
    setColor(255, 0, 0); // Red
  } else if (greenFrequency > redFrequency && greenFrequency > blueFrequency) {
    // Green color detected
    setColor(0, 255, 0); // Green
  } else if (blueFrequency > redFrequency && blueFrequency > greenFrequency) {
    // Blue color detected
    setColor(0, 0, 255); // Blue
  } else {
    // No dominant color or uncertain
    setColor(255, 255, 255); // White or turn off LED
  }

  Serial.println("R:");
  Serial.println(redFrequency);
  Serial.println("G:");
  Serial.println(greenFrequency);
  Serial.println("B:");
  Serial.println(blueFrequency);
  
  delay(1000); // Delay for a bit before reading again
}

void readColorFrequency() {
  // Reading RED
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  redFrequency = pulseIn(sensorOut, LOW);
  
  // Reading GREEN
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
  
  // Reading BLUE
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);
}

void setColor(int red, int green, int blue) {
  // Assuming common anode RGB LED
  analogWrite(RED_LED, 255 - red);
  analogWrite(GREEN_LED, 255 - green);
  analogWrite(BLUE_LED, 255 - blue);
}
