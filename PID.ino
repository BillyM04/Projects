#include <WiFi.h>
#include <HTTPClient.h>

// Motor pins
  int m1PWM = 37; // m1 left
  int m1PH = 38;
  int m2PWM = 39; // m2 right
  int m2PH = 20;
  int mode = 36;
// Speed initializations
  const int forward_base_speed = 200;
  const int reverse_base_speed = 200;

// Sensor pins
  float error = 0;
  int AnalogPin[6] = {4, 5, 6, 7, 15,16};
  int sensorILeft = analogRead(AnalogPin[1]);
  int sensorIRight = analogRead(AnalogPin[3]);
  int sensorOLeft = analogRead(AnalogPin[0]);
  int sensorORight = analogRead(AnalogPin[4]);
  int sensorCenter = analogRead(AnalogPin[2]);
  int obj_det = analogRead(AnalogPin[5]);
  int line_threshold = 700;
  int obj_threshold = 300;

// PID constants and variables
  float  kp = 3;
  float kd = 1.1;
  float lastError = 0;

//path planning
  int da[8][8] = {{0,0,0,0,2,0,3,0},
                  {0,0,0,0,0,0,1,1},
                  {0,0,0,2,0,0,3,0},
                  {0,0,2,0,0,0,0,4},
                  {2,0,0,0,0,0,0,4},
                  {0,0,0,0,0,0,0,0},
                  {3,1,3,0,0,0,0,0},
                  {0,1,0,4,4,0,0,0}} 


void setup() {
  Serial.begin(9600);

  //pins for motor
    pinMode(mode, OUTPUT);
    pinMode(m1PWM, OUTPUT);
    pinMode(m1PH, OUTPUT);
    pinMode(m2PWM, OUTPUT);
    pinMode(m2PH, OUTPUT);
    //digitalWrite(mode, HIGH);

    pinMode(AnalogPin[0], INPUT);
    pinMode(AnalogPin[1], INPUT);
    pinMode(AnalogPin[2], INPUT);
    pinMode(AnalogPin[3], INPUT);
    pinMode(AnalogPin[4], INPUT);
    pinMode(AnalogPin[5], INPUT);
  }
//sensor from 0-4096 to 0-1000
int normalizeSensor(int rawValue) {
  const int MIN_SENSOR_VALUE = 0; // Approx range for white
  const int MAX_SENSOR_VALUE = 4096; // Approx range for black
  rawValue = constrain(rawValue, MIN_SENSOR_VALUE, MAX_SENSOR_VALUE); // Constrain to range
  return map(MAX_SENSOR_VALUE - rawValue, 0, 4096, 0, 1000); // Normalize
}
//line error
float calculateError(int sensorILeft, int sensorCenter, int sensorIRight) {
  int total = sensorILeft + sensorCenter + sensorIRight;
  if (total == 0) {
    return 0; 
  }
  float error = (-1.0 * sensorILeft + 1.0 * sensorIRight) / total;
  //debug prints
    // Serial.print("\n\n");
    // Serial.print(sensorILeft);
    // Serial.print("\t");
    // Serial.print(sensorCenter);
    // Serial.print("\t");
    // Serial.print(sensorIRight);
    // Serial.print("\n\n");
  return error;
}
// PID
float computePID(float error) {
  float P = kp * error;
  float D = kd * (error - lastError);
  lastError = error;
  return P + D;
}
//move forward
void forward(int baseSpeed, float correction) {
  int leftMotorSpeed = baseSpeed - int(correction * 200);
  int rightMotorSpeed = baseSpeed + int(correction * 200);
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 250);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 250);

  digitalWrite(mode, HIGH);
  digitalWrite(m1PH, LOW); // Left motor forward
  digitalWrite(m2PH, HIGH);  // Right motor forward
  analogWrite(m1PWM, leftMotorSpeed);
  analogWrite(m2PWM, rightMotorSpeed);

  Serial.print(leftMotorSpeed);
  Serial.print("\t");
  Serial.print(rightMotorSpeed);
  Serial.print("\n");
}
//brake
void brake(int time_delay){
  digitalWrite(mode, HIGH);
  digitalWrite(m1PH, LOW); // Left motor forward
  digitalWrite(m2PH, HIGH);  // Right motor forward
  analogWrite(m1PWM, 0);
  analogWrite(m2PWM, 0);
  delay(time_delay);
}
//180 turn
void half_turn(){
  digitalWrite(mode, HIGH);
  digitalWrite(m1PH, HIGH); // Left motor forward
  digitalWrite(m2PH, HIGH);  // Right motor forward
  analogWrite(m1PWM, 180);
  analogWrite(m2PWM, 180);
}

void loop() {
  //debug prints
    // Serial.print("S1\tS2\tS3\n");
    // Serial.print(normalizeSensor(analogRead(AnalogPin[1])));
    // Serial.print("\t");
    // Serial.print(normalizeSensor(analogRead(AnalogPin[2]))); 
    // Serial.print("\t"); 
    // Serial.print(normalizeSensor(analogRead(AnalogPin[3])));
    // Serial.print("\n"); 
    // Serial.print("\nerror:\t");
    // Serial.print(error);
    // Serial.print("\n\n");
  error = calculateError(normalizeSensor(analogRead(AnalogPin[1])), normalizeSensor(analogRead(AnalogPin[2])), normalizeSensor(analogRead(AnalogPin[3])));  
  forward(forward_base_speed, computePID(error));

  if(normalizeSensor(analogRead(AnalogPin[0]))>= line_threshold && normalizeSensor(analogRead(AnalogPin[1]))>= line_threshold && normalizeSensor(analogRead(AnalogPin[2]))>= line_threshold && normalizeSensor(analogRead(AnalogPin[3]))>= line_threshold && normalizeSensor(analogRead(AnalogPin[4]))>= line_threshold){
    brake(1000);
  }

  if(normalizeSensor(analogRead(AnalogPin[5]))<=obj_threshold){  
    brake(2000);
    half_turn();
    delay(500);
    while(normalizeSensor(analogRead(AnalogPin[2])) <= line_threshold){
      half_turn();
    }
  }
  delay(25);
  //debug prints
    // Serial.print(normalizeSensor(analogRead(AnalogPin[0]))); 
    // Serial.print("\t"); 
    // Serial.print(normalizeSensor(analogRead(AnalogPin[1])));
    // Serial.print("\t");
    // Serial.print(normalizeSensor(analogRead(AnalogPin[2]))); 
    // Serial.print("\t"); 
    // Serial.print(normalizeSensor(analogRead(AnalogPin[3])));
    // Serial.print("\t"); 
    // Serial.print(normalizeSensor(analogRead(AnalogPin[4])));
    // Serial.print("\t");
    // Serial.print(normalizeSensor(analogRead(AnalogPin[5])));
    // Serial.println("");
    // delay(600);
}
