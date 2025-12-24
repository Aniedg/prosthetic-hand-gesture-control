#include <Servo.h>

Servo thumb_servo;
Servo index_servo;
Servo middle_servo;
Servo ring_servo;
Servo pinky_servo;

const int THUMB_PIN = 3;
const int INDEX_PIN = 5;
const int MIDDLE_PIN = 6;
const int RING_PIN = 9;
const int PINKY_PIN = 10;

const int SERVO_MIN = 0;
const int SERVO_MAX = 180;

int thumbAngle = 90;
int indexAngle = 90;
int middleAngle = 90;
int ringAngle = 90;
int pinkyAngle = 90;

const int UPDATE_DELAY = 20;
unsigned long lastUpdateTime = 0;

const int BUFFER_SIZE = 64;
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;

const int DEBUG_MODE = 0;

void setup() {
  Serial.begin(9600);
  
  thumb_servo.attach(THUMB_PIN);
  index_servo.attach(INDEX_PIN);
  middle_servo.attach(MIDDLE_PIN);
  ring_servo.attach(RING_PIN);
  pinky_servo.attach(PINKY_PIN);
  
  thumb_servo.write(90);
  index_servo.write(90);
  middle_servo.write(90);
  ring_servo.write(90);
  pinky_servo.write(90);
  
  delay(500);
  
  Serial.println("Arduino Ready");
  Serial.println("Waiting for servo commands...");
  
  if (DEBUG_MODE) {
    Serial.println("DEBUG MODE ENABLED");
  }
}

void loop() {
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    
    if (incoming != '\n') {
      if (bufferIndex < BUFFER_SIZE - 1) {
        inputBuffer[bufferIndex] = incoming;
        bufferIndex++;
      }
    } else {
      inputBuffer[bufferIndex] = '\0';
      
      if (bufferIndex > 0) {
        if (parseAndUpdate(inputBuffer)) {
          updateServos();
          
          if (DEBUG_MODE) {
            Serial.print("âœ“ Updated: T:");
            Serial.print(thumbAngle);
            Serial.print(" I:");
            Serial.print(indexAngle);
            Serial.print(" M:");
            Serial.print(middleAngle);
            Serial.print(" R:");
            Serial.print(ringAngle);
            Serial.print(" P:");
            Serial.println(pinkyAngle);
          }
        } else {
          Serial.println("âœ— Parse error. Expected format: T:XX,I:XX,M:XX,R:XX,P:XX");
        }
      }
      
      bufferIndex = 0;
    }
  }
  
  delay(5);
}

bool parseAndUpdate(char* data) {
  char buffer[BUFFER_SIZE];
  strcpy(buffer, data);
  
  int t = -1, i = -1, m = -1, r = -1, p = -1;
  char* ptr = buffer;
  
  if (buffer[0] == 'T' && buffer[1] == ':') {
    t = atoi(&buffer[2]);
  }
  
  ptr = strchr(buffer, 'I');
  if (ptr != NULL && *(ptr + 1) == ':') {
    i = atoi(ptr + 2);
  }
  
  ptr = strchr(buffer, 'M');
  if (ptr != NULL && *(ptr + 1) == ':') {
    m = atoi(ptr + 2);
  }
  
  ptr = strchr(buffer, 'R');
  if (ptr != NULL && *(ptr + 1) == ':') {
    r = atoi(ptr + 2);
  }
  
  ptr = strchr(buffer, 'P');
  if (ptr != NULL && *(ptr + 1) == ':') {
    p = atoi(ptr + 2);
  }
  
  if (t == -1 || i == -1 || m == -1 || r == -1 || p == -1) {
    return false;
  }
  
  if (t < SERVO_MIN || t > SERVO_MAX ||
      i < SERVO_MIN || i > SERVO_MAX ||
      m < SERVO_MIN || m > SERVO_MAX ||
      r < SERVO_MIN || r > SERVO_MAX ||
      p < SERVO_MIN || p > SERVO_MAX) {
    Serial.println("âœ— Angle out of range (0-180)");
    return false;
  }
  
  thumbAngle = t;
  indexAngle = i;
  middleAngle = m;
  ringAngle = r;
  pinkyAngle = p;
  
  return true;
}

void updateServos() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime < UPDATE_DELAY) {
    return;
  }
  lastUpdateTime = currentTime;
  
  thumb_servo.write(thumbAngle);
  index_servo.write(indexAngle);
  middle_servo.write(middleAngle);
  ring_servo.write(ringAngle);
  pinky_servo.write(pinkyAngle);
}
