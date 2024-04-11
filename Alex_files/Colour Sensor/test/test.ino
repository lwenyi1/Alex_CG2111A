// Pin assignments for S0, S1, S2, S3, and OUT pins
#define S0 6
#define S1 5
#define S2 4
#define S3 3
#define sensorOut 2

int frequency = 0;

void setupColour() { //baremetal when got time
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
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

void evaluateColour(int r, int g, int b) {
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
    colour.params[3] = 0; //RED
  } else if (weightedGreen > weightedWhite) {
    colour.params[3] = 1; //GREEN
  } else {
    colour.params[3] = 2; //WHITE
  }
  return;
}

void readColour() {
  TPacket colour;
  colour.packetType = PACKET_TYPE_RESPONSE;
  colour.command = RESP_COLOURSENSOR;
  
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  colour.params[0] = frequency;

  delay(100);

  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  colour.params[1] = frequency;

  delay(100);

  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  colour.params[2] = frequency;
  
  delay(100);

  evaluateColour(colour.params[0], colour.params[1], colour.params[2]);

  sendResponse(&colour);
}


void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  
  Serial.begin(9600);
}

void loop() {
  readColour();
}