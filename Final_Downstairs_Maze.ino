#include <Wire.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>

#define CLK 36  // CLK for Encoder 1
#define DT 39   // DT for Encoder 1
#define CLK2 35 // CLK for Encoder 2
#define DT2 34  // DT for Encoder 2

ESP32Encoder encoder;   // Create rotary encoder object encoder (1st encoder)
ESP32Encoder encoder2;  // Create rotary encoder object encoder2 (2nd encoder)

Servo myservo;            // Create servo object myservo
int servoPin = 13;        // Servo pin on downstairs ESP32
int steeringAngle = 90;              // Initialise servo position

// L298N Motor Driver pins
const int motorPin1 = 26;  // IN_A pin on downstairs ESP32 (according to schematic); Direction for Motor 1
const int motorPin2 = 27;  // IN_B pin on downstairs ESP32 (according to schematic); Direction for Motor 1
const int motorPin3 = 14;  // IN_C pin on downstairs ESP32 (according to schematic); Direction for Motor 2
const int motorPin4 = 12;  // IN_D pin on downstairs ESP32 (according to schematic); Direction for Motor 2
const int enablePinA = 33; // EN_A pin on downstairs ESP32 (according to schematic); PWM inputs for speed control of Motor 1
const int enablePinB = 25; // EN_B pin on downstairs ESP32 (according to schematic); PWM inputs for speed control of Motor 2

// Setting PWM properties for motors
const int freq = 2000;
const int pwmChannela = 0;  // PWM channel allocation for speed control of Motor 1
const int pwmChannelb = 1;  // PWM channel allocation for speed control of Motor 2
const int resolution = 8;
int dutyCycle = 140;

int yaw; // Initialise yaw globally to ensure it is accessible for all functions

// For KeyPad
char Key; // Key variable intialised globally to ensure it updates globally
char commands[20]; // Array for holding commands entered to be processed; maximum of 20 commands
int commandIndex = 0; // Command index intialised and set to 0

// For distance from encoders
const float wheelCircumference = 21.6; // EEEBot rear wheel circumference in cm
const int encoderCPR = 47;             // Counts per revolution; determined empirically (rather than from datasheet)
const float pulsePerCM = wheelCircumference / encoderCPR; // Pulse per centimeter; multiply by counts to get distance in cm

int P, I, D, previousError, PID;
int setpoint = 0; // Setpoint intialised to 0
int kP = 2; // Determined emprically
int kI = 0;
int kD = 0;

bool newDataAvailable = false; // Initalise signal for beginning command processing
bool yawUpdated = false; // Initialise signal for controlling steering
bool pidEnabled = true; // Initialise signal for activating PID control of steering
int currentActivity = 14; // Initialise signal for determining current activity of EEEBot for communication with upstairs esp32 (on request)

void setup() 
{
  Wire.begin(8);                // I2C slave address
  Wire.onRequest(onRequest);  // For data requested by upstairs (master device) esp32; the upstairs esp32 requests turn completion data in this code
  Wire.onReceive(receiveData); // For data received from upstairs (master device) esp32; yaw (as two isolated bytes) and key
  Serial.begin(9600);

  // Setting up motor control pins as outputs to control direction of EEEbot (motorPin1-4) and speed (enablePin1-2)
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(enablePinA, OUTPUT);
  pinMode(enablePinB, OUTPUT);

  // Setting up PWM channels for motor speed control
  ledcSetup(pwmChannela, freq, resolution);
  ledcSetup(pwmChannelb, freq, resolution);
  
  // Attaching enable pins to PWM channels for motor speed control
  ledcAttachPin(enablePinA, pwmChannela); // pwmChannela will define motor speed for motor 1
  ledcAttachPin(enablePinB, pwmChannelb); // pwmChannelb will define motor speed for motor 2

  ESP32PWM::allocateTimer(2);    // Allocate timer for PWM control of servo (timers 0 and 1 of ESP32 already occupied by pwmChannela and pwmChannelb, respectively)
  myservo.setPeriodHertz(50);    // Standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // Attach servo (myservo) to control pin (13) according to schematic}

  encoder.attachHalfQuad(DT, CLK);    // Attach encoder 1 to DT and CLK pins on ESP32
  encoder2.attachHalfQuad(DT2, CLK2); // Attach encoder 2 to DT and CLK pins on ESP32
  encoder.setCount(0);  // Initialise encoder 1 count to 0
  encoder2.setCount(0); // Initialise encoder 2 count to 0  
}

void loop() 
{
  if (yawUpdated) // If yaw data received from upstairs esp32
  {
    //Serial.print("yaw: "); Serial.println(yaw);
    updateSteering(); // Use yaw to control steering via PID
    yawUpdated = false; // Reset flag
  }

  if (newDataAvailable) // If all commands are ready to be processed 
  {
    for (int i = 0; i < commandIndex; i++) // Iterate through each index of the array
    {
    // char currentCommand = commands[i];
    processCommand(commands[i]); // Process commands one-by-one, through the array
    }

    // Reset character array holding commands and clear buffer to prepare for next set of commands
    commandIndex = 0;
    memset(commands, 0, sizeof(commands));
    newDataAvailable = false;
  }
}

void temporaryDisablePID() { // Function for disabling PID pre-turns
    pidEnabled = false;
}

void restorePID() { // Function for enabling PID post-turns
    pidEnabled = true;
}

void updateSteering() // Incorporating PID using the MPU6050 sensor data, to control steering angle when driving straight
{
  if (!pidEnabled) return; // If PID is disabled exit and do not update steering
  
  else // If PID is enabled map PID value on steering angle
  {
  int error = setpoint - yaw;
  P = error;
  I = I + error;
  D = error - previousError;

  PID = (kP * P) + (kI * I) + (kD * D);
  previousError = error;

  // Serial.println(PID);

  steeringAngle = map(PID, -50, 50, 180, 0); // Map PID onto steering angle
  myservo.write(steeringAngle);
  }
}

void processCommand(char currentCommand) // Perform action depending on specific command and use 'currentActivity' to tell the upstairs esp32 what the downstairs esp32 is doing
{
    switch (currentCommand) 
    {
      case '0':
        stopMotors();
        currentActivity = 0;
        break;

      case '1':
        currentActivity = 1;
        go10();  
        currentActivity = 14;    
        break;

      case '2':
        currentActivity = 2;
        go20();
        currentActivity = 14;
        break;

      case '3':
        currentActivity = 3;
        go30();
        currentActivity = 14;
        break;

      case '4':
        currentActivity = 4;
        go40();
        currentActivity = 14;
        break;

      case '5':
        currentActivity = 5;
        go50();
        currentActivity = 14;
        break;

      case '6':
        currentActivity = 6;
        go60();
        currentActivity = 14;
        break;

      case '7':
        currentActivity = 7;
        go70();
        currentActivity = 14;
        break;

      case '8':
        currentActivity = 8;
        go80();
        currentActivity = 14;
        break;

      case '9':
        currentActivity = 9;
        go90();
        currentActivity = 14;
        break;

      case '*':
        currentActivity = 10;
        temporaryDisablePID(); // Disable PID before turn to prevent PID interference with steering angle
        turnLeft();
        restorePID(); // Restore PID after turn completed
        currentActivity = 13;
        break;

      case '#':
        currentActivity = 11;
        temporaryDisablePID(); // Disable PID before turn to prevent PID interference with steering angle
        turnRight();
        restorePID(); // Restore PID after turn completed
        currentActivity = 13;
        break;
    }
}

void receiveData(int howMany) 
{
  if (Wire.available() >= 3) // Expecting 3 bytes (high byte of yaw, low byte of yaw, and key)
  { 
    int highByte = Wire.read(); // High byte of yaw
    int lowByte = Wire.read();  // Low byte of yaw
    yaw = (highByte << 8) | lowByte; // Recombine bytes to form yaw angle measured by MPU6050
    if (highByte & 0x80) 
    {
      yaw = yaw | 0xFFFF0000; // For accommodating negative values of yaw
    }
    yawUpdated = true; // Signal for steering to be controlled via PID (see loop function)

    Key = Wire.read(); // Read the key (or placeholder if no key pressed)
    if (Key != 0xFF && commandIndex < 20) // If the key was not a placeholder and the character array for storing commands isn't full
    {
      commands[commandIndex++] = Key; // Store pressed key in character array
    }

    if (commandIndex >= 20 || Key == '0') // if the character array is fully populated or the '0' key was pressed
    {
      newDataAvailable = true; // Signal command processing via character array (see loop function)
    }
  }
}

void goDistance(int targetDistance) // Function to travel for a specified distance in a straight line
{
  delay(8000); // Delay mainly to accommodate turns, so that the sensor can reinitialise before proceeding
  setpoint = yaw; // set the setpoint to the yaw when the EEEBot is stable/not moving
  encoder.setCount(0);  // Ensures encoder 1 is set to 0
  encoder2.setCount(0); // Ensures encoder 2 is set to 0

  // Convert target distance to target encoder counts
  // long targetCounts = targetDistance / pulsePerCM; // For encoder calibration purposes

  goForward(); // Start moving forward

  while(true)
  {
    long count1 = encoder.getCount();   // Current count for encoder 1
    long count2 = encoder2.getCount();  // Current count for encoder 2

    // Calculate the average distance based on encoder counts
    float averageDistance = ((count1 + count2) / 2.0) * pulsePerCM;

    if (pidEnabled) // For ensuring straight driving
    {
      updateSteering(); // Continuously adjust steering based on current yaw using PID
    }
  
    // Check if the vehicle has reached or exceeded the target distance
    if (averageDistance >= targetDistance) 
    {
      stopMotors(); // Stop motors
      break; // Exit the loop
    }
    delay(10);
  }
  if (pidEnabled) // Post-drive
  {
    myservo.write(90); // Center the servo if PID is still considered active, this can probably be commented out
  }
}

void goForward()
{
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW); 
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW); 

  ledcWrite(pwmChannela, 140);
  ledcWrite(pwmChannelb, 140);
}

void turnLeft() 
{
  encoder.setCount(0);  // Ensuring encoder 1 is reinitialised to 0
  encoder2.setCount(0); // Ensuring encoder 2 is reinitialised to 0
  myservo.write(0); 

  // Convert target distance to target encoder counts
  // long targetCounts = 44 / pulsePerCM; // for encoder calibration purposes

  goForward(); // Start moving forward

  while(true)
  {
    long count2 = encoder2.getCount();  // Current count for encoder 2

    // Calculate the average distance based on encoder counts
    float encTwoDistance = count2 * pulsePerCM;

    // Check if the vehicle has reached or exceeded the target distance (determined empirically)
    if (encTwoDistance >= 51) 
    {
      stopMotors(); // Stop motors
      encoder.setCount(0);  // Reset encoder counts to 0
      encoder2.setCount(0); // Reset encoder counts to 0
      break; // Exit the loop
    }
    delay(10);
  }
  //currentActivity = 13; // Just here in case issues with this being set in the command processing function
}

void turnRight()
{
  encoder.setCount(0);  // Ensuring encoder is reset to 0
  encoder2.setCount(0); // Ensuring encoder is reset to 0

  myservo.write(180);

  // Convert target distance to target encoder counts
  // long targetCounts = 32 / pulsePerCM; // for encoder calibration purposes

  goForward(); // Start moving forward

  while(true)
  {
    long count1 = encoder.getCount();   // Current count for encoder 1; using encoder 1 for right turn measurement

    // Calculate the average distance based on encoder counts
    float encOneDistance = count1 * pulsePerCM;

    // Check if EEEBot has reached or exceeded the target distance (calue determined empirically)
    if (encOneDistance >= 43) 
    {
      stopMotors(); // Stop motors
      encoder.setCount(0);  // Reset encoder 1
      encoder2.setCount(0); // Reset encoder 2
      break; // Exit the loop
    }
    delay(10);
  }
  //currentActivity = 13; // Signal for turn completion, sent to upstairs esp32; triggers MPU6050 reinitialisation
}


void stopMotors() // Stops motors/EEEBot
{
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  myservo.write(90);
}

void motorSpeed(int leftSpeed, int rightSpeed) // For setting motor speed
{
  ledcWrite(pwmChannela, leftSpeed);
  ledcWrite(pwmChannelb, rightSpeed);
}

// Various functions for determining distance
void go10()
{
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(7);
}

void go20()
{
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(18);
}

void go30()
{
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(27);
}

void go40()
{
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(36);
}

void go50()
{
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(48);
}

void go60()
{
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(57);
}

void go70()
{
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(65);
}

void go80()
{
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(74);
}

void go90()
{
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(83);
}

void onRequest() // For sending data to the upstairs esp32 to trigger sensor reinitialisation; upstairs esp32 is requesting data constantly - one of the below must be sent
{
    if (currentActivity == 0) {Wire.write(0); currentActivity = 14;} // Motors stopped
    if (currentActivity == 1) {Wire.write(1);} // Going 10cm
    if (currentActivity == 2) {Wire.write(2);} // Going 20cm
    if (currentActivity == 3) {Wire.write(3);} // Going 30cm
    if (currentActivity == 4) {Wire.write(4);} // Going 40cm
    if (currentActivity == 5) {Wire.write(5);} // Going 50cm
    if (currentActivity == 6) {Wire.write(6);} // Going 60cm
    if (currentActivity == 7) {Wire.write(7);} // Going 70cm
    if (currentActivity == 8) {Wire.write(8);} // Going 80cm
    if (currentActivity == 9) {Wire.write(9);} // Going 90cm
    if (currentActivity == 10) {Wire.write(10);} // Turning left
    if (currentActivity == 11) {Wire.write(11);} // Turning right
    if (currentActivity == 12) {Wire.write(12);} // Extra (not application yet)
    if (currentActivity == 13) {Wire.write(13); currentActivity = 14;} // reinitialise sensors
    if (currentActivity == 14) {Wire.write(14);} // Display nothing
}