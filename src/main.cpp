#include <Arduino.h>
#include "IMU.h"
#include <Adafruit_PWMServoDriver.h>
#include <string.h>
#include <stdio.h>
#include <EEPROM.h>
#include <math.h>
#include <util/atomic.h>
// #include "SparkFun_VL53L1X.h"
/*
PIN ASSIGNMENTS:
Motor Right --> D1 & D2
Motor Left  --> D3 & D4
Scissor Lift --> D5 & D6
Scoop Servo --> D9
Containment Gate --> D10
Swing Arm Right --> D11
Swing Arm Left  --> D12
*/
// PWM CONTROLLER VARIABLES

#define SCISSOR_LIFT_UP 2
#define SCISSOR_LIFT_DOWN 3

#define ARM_SPEED 50
#define SERVO_MIN 150
#define SERVO_MAX 600

#define MOTOR_RIGHT_IN1 8
#define MOTOR_RIGHT_IN2 9
#define MOTOR_RIGHT_EN 10

#define MOTOR_LEFT_IN1 4
#define MOTOR_LEFT_IN2 5
#define MOTOR_LEFT_EN 6

#define IMU_START_DELAY 1000
#define IMU_END_DELAY 1000
#define ROTATION_TOLERANCE 1.0

struct orientation {
  int time;
  float yaw;
  float pitch;
  float roll;
  int motorRight;
  int motorLeft;
};

typedef enum
{
  START,
  MANUAL,
  AUTO,
  CALIBRATE,
} robotMode;

typedef enum
{
  SCOOP,
  LIFTLEFT,
  LIFTRIGHT,
  GATE,
  CONTAINMENT,
} servoID;

// I2C Devices
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();
IMU myIMU;
// SFEVL53L1X distanceSensor;


//IMU data recording
#define IMU_DATA_SIZE 300
int IMUDataCount = 0;
orientation IMUData[IMU_DATA_SIZE];


char command[32];
char commandType[5];
char direction;

int timeval = 0;
int value = 0;
int speedval = 0;
int currentArmAngle = 180;
int servoRightAngle = 165;
int servoLeftAngle = 75;
int pulselength = 0;
int pulselength2 = 0;
int pwmOutput = 0;
int deltaAngle = 0;
int containmentAngle = 30;
int mode = robotMode::START;
volatile int drivetime;
volatile int sampletime;


double y = 0;
double p = 0;
double r = 0;

double yaw_Kp = 1.1;
double yaw_Ki = 0.5;
double yaw_integral = 0;



// Pins
const int enca[] = {18,19};
const int encb[] = {12,13};
const int pwm[] = {10,6};
const int in1[] = {8,4};
const int in2[] = {9,5};

// Motor Globals
volatile long pos_Right_i = 0;
volatile long pos_Left_i  = 0;

int kp_r = 4;
int ki_r = 11;

int kp_l = 4;
int ki_l = 12;


void executeCommand();
void scissorLiftActuate(int direction, int duration);
void scoop(int angle);
void arm(int speed, int angle);
void gate(int angle);
void forwardBack(int direction, int time, int speed);
void rotate(int direction, double angle, int speed);
void runMission();

//Encoder Motor Control
void move(int direction, int time, int speed);
void init_motor();
void motor(int revs, int speed);
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void setTarget(float t, float deltat);
void readEncoderRight();
void readEncoderLeft();
// int getdistance();
//Data Collection
void dumpMemory(uint16_t stop);
void recordYaw(double& yaw, double& pitch, double& roll,int right,int left);
void outputYaw();


void setup()
{
  Serial.begin(115200);
  Serial.println("SYSTEM STARTING!!!");

  myIMU.init_imu();
  init_motor();

  servos.begin();
  servos.setPWMFreq(60);
  servos.setPWM(servoID::LIFTLEFT, 0, map(servoLeftAngle, 0, 270, SERVO_MIN, SERVO_MAX));
  servos.setPWM(servoID::LIFTRIGHT, 0, map(servoRightAngle, 0, 270, SERVO_MIN, SERVO_MAX));
  servos.setPWM(servoID::SCOOP, 0, map(140, 0, 270, SERVO_MIN, SERVO_MAX));
  servos.setPWM(servoID::GATE, 0, map(90, 0, 270, SERVO_MIN, SERVO_MAX));
  servos.setPWM(servoID::CONTAINMENT, 0, map(containmentAngle, 0, 180, SERVO_MIN, SERVO_MAX));
  pinMode(SCISSOR_LIFT_UP, OUTPUT);
  pinMode(SCISSOR_LIFT_DOWN, OUTPUT);
  sei();
  Serial.println("SYSTEM INITIALIZED!!!");
}

void loop()
{
  if (mode == robotMode::START)
  {
    Serial.readBytes(command, 16);
    sscanf(command, "%3s", commandType);
    if ((strcmp(commandType, "MAN")) == 0)
    {
      Serial.println("Manual Mode Activated");
      mode = robotMode::MANUAL;
    }
    else if ((strcmp(commandType, "GO!")) == 0)
    {
      mode = robotMode::AUTO;
    }
    else if ((strcmp(commandType, "CAL")) == 0)
    {
      Serial.println("Commencing IMU Calibration (will take 2 Minutes)");
      myIMU.calibrate_imu();
      Serial.println("Calibration Complete!");
      mode = robotMode::MANUAL;
    }
    else if ((strcmp(commandType, "DMP")) == 0)
    {
      sscanf(command, "%*3s %04d", &value);
      dumpMemory(value);
    }
  }
  else if (mode == robotMode::MANUAL)
  {
    executeCommand();
  }
  else if (mode == robotMode::AUTO)
  {
    runMission();
  }
  commandType[0] = '\0';
  command[0] = '\0';
}

void runMission()
{

}

void executeCommand()
{
  Serial.readBytes(command, 32);
  sscanf(command, "%3s", commandType);
  // Scoop Actuate:    SCP [Angle] ---> "SCP 210"
  if (strcmp(commandType, "SCP") == 0)
  {
    command[strlen(command) + 1] ='\0';
    Serial.println(command);
    sscanf(command, "%*3s %04d", &value);
    // pulselength = map(value, 0, 270, SERVO_MIN, SERVO_MAX);
    // servos.setPWM(servoID::SCOOP, 0, pulselength);
    scoop(value);
  }
  // Containment Gate: GTE [Direction +|-] --->                 "GTE + Closes - Opens"
  else if (strcmp(commandType, "GTE") == 0)
  {
    sscanf(command, "%*3s %c", &direction);
    if (direction == '+')
    {
      // pulselength = map(90, 0, 180, SERVO_MIN, SERVO_MAX);
      // servos.setPWM(servoID::GATE, 0, pulselength);
      gate(90);
    }
    else if (direction == '-')
    {
      // pulselength = map(0, 0, 180, SERVO_MIN, SERVO_MAX);
      // servos.setPWM(servoID::GATE, 0, pulselength);
      gate(0);
    }
  }
  // Scissor Lift:     SCI [Direction +|-] [Duration (ms)] ---> "SCI + 10000"
  else if (strcmp(commandType, "SCI") == 0)
  {
    sscanf(command, "%*3s %c %04d", &direction, &timeval);
    digitalWrite(SCISSOR_LIFT_UP, LOW);
    digitalWrite(SCISSOR_LIFT_DOWN, HIGH);
    Serial.println(command);
    if (direction == '+')
    {
      scissorLiftActuate(1, timeval * 1000);
    }
    else if (direction == '-')
    {
      scissorLiftActuate(-1, timeval * 1000);
    }
  }
  // For/Back:         MOV [Direction +|-] [Revolutions] [Speed (RPM) 0 < X < 1000] ---> "MOV + 2 10"
  else if (strcmp(commandType, "MOV") == 0)
  {
    sscanf(command, "%*3s %c %04d %04d %04d %04d %04d %04d", &direction ,&timeval, &speedval,&kp_r,&ki_r,&kp_l,&ki_l);
    command[strlen(command) + 1] ='\0';
    Serial.println("");
    Serial.println(command);

    //Reset Array each movement
    for (int i = 0;i < IMUDataCount;i++){
      IMUData[i].time = 0;
      IMUData[i].yaw = 0;
      IMUData[i].pitch = 0;
      IMUData[i].roll = 0;
      IMUData[i].motorLeft = 0;
      IMUData[i].motorRight = 0;
    }
    IMUDataCount = 0;

    if (direction == '+'){
      //forwardBack(1,time,speedval);
      move(1,timeval,speedval);
      
    }
    else if (direction == '-'){
      //forwardBack(-1,time,speedval);
      move(-1,timeval,speedval);
    }
  }
  // Rotate:           ROT [Direction +|-] [Angular Displacement] [Speed (RPM) 0 < X < 1000] ---> "ROT - 3 30"
  else if (strcmp(commandType, "ROT") == 0)
  {
    sscanf(command, "%*3s %c %04d %04d", &direction, &deltaAngle , &speedval);

    //Reset Array each movement
    for (int i = 0;i < IMUDataCount;i++){
      IMUData[i].time = 0;
      IMUData[i].yaw = 0;
      IMUData[i].pitch = 0;
      IMUData[i].roll = 0;
      IMUData[i].motorRight = 0;
      IMUData[i].motorLeft = 0;
    }
    IMUDataCount = 0;


    if (direction == '+')
    {
      rotate(1, (double) deltaAngle,speedval);
    }
    else if (direction == '-')
    {
      rotate(-1, (double) deltaAngle,speedval);
    }
  }
  // ARM:              ARM [SPEED] [ANGLE] ---> "ARM 50 85"
  else if (strcmp(commandType, "ARM") == 0)
  {
    sscanf(command, "%*3s %04d %04d", &speedval, &value);
    Serial.println(command);
    arm(speedval,value);
  }
  else if (strcmp(commandType, "CON") == 0)
  {
    command[strlen(command) + 1] ='\0';
    Serial.println(command);
    sscanf(command, "%*3s %04d", &value);
    containmentAngle = value;
    pulselength = map(value, 0, 180, SERVO_MIN, SERVO_MAX);
    servos.setPWM(servoID::CONTAINMENT, 0, pulselength);
  }
  // Get Distance Sensor Reading: "DIS"
  //  else if (strcmp(commandType,"DIS") == 0){
  //      Serial.print("Distance(mm): ");
  //      Serial.print(getdistance());
  //  }
  // Get IMU Yaw Reading: "YAW"
  else if (strcmp(commandType, "YAW") == 0)
  {
    outputYaw();
  }
  else if (strcmp(commandType, "MTR") == 0)
  {
    sscanf(command, "%*3s %c %04d %04d %04d %04d %04d %04d", &direction ,&timeval, &speedval,&kp_r,&ki_r,&kp_l,&ki_l);
    if (direction == '+')
    {
      move(1,timeval,speedval);
    }
    else if (direction == '-')
    {
      move(-1,timeval,speedval);
    }
  }
  commandType[0]='\0';
  command[0] = '\0';
}

void scissorLiftActuate(int direction, int duration)
{
  // Move Lift Up
  if (direction == 1)
  {
    digitalWrite(SCISSOR_LIFT_UP, HIGH);
    digitalWrite(SCISSOR_LIFT_DOWN, LOW);
    delay(duration);
    digitalWrite(SCISSOR_LIFT_UP, LOW);
    digitalWrite(SCISSOR_LIFT_DOWN, LOW);
  }
  // Move Lift Down
  else if (direction == -1)
  {
    digitalWrite(SCISSOR_LIFT_DOWN, HIGH);
    digitalWrite(SCISSOR_LIFT_UP, LOW);
    delay(duration);
    digitalWrite(SCISSOR_LIFT_DOWN, LOW);
    digitalWrite(SCISSOR_LIFT_UP, LOW);
  }
}

// int getdistance(){
//   distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
//   while (!distanceSensor.checkForDataReady())
//   {
//     delay(1);
//   }
//   int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
//   distanceSensor.clearInterrupt();
//   distanceSensor.stopRanging();
//   return distance;
// }

void arm(int speed, int angle)
{
  if (servoRightAngle > angle)
  {
    //Arm Comes Out
    for (int i = servoRightAngle; i > angle; i--)
    {
      //containmentAngle++;
      servoLeftAngle++;
      servoRightAngle--;
      pulselength = map(servoLeftAngle, 0, 270, SERVO_MIN, SERVO_MAX);
      pulselength2 = map(servoRightAngle, 0, 270, SERVO_MIN, SERVO_MAX);
      servos.setPWM(servoID::LIFTLEFT, 0, pulselength);
      servos.setPWM(servoID::LIFTRIGHT, 0, pulselength2);
      //servos.setPWM(servoID::CONTAINMENT,0,map(containmentAngle,0,180,SERVO_MIN,SERVO_MAX));
      delay(speed);
    }
  }
  else
  {
    //Arm Goes Down.
    for (int i = servoRightAngle; i < angle; i++)
    {
      //containmentAngle--;
      servoLeftAngle--;
      servoRightAngle++;
      pulselength = map(servoLeftAngle, 0, 270, SERVO_MIN, SERVO_MAX);
      pulselength2 = map(servoRightAngle, 0, 270, SERVO_MIN, SERVO_MAX);
      servos.setPWM(servoID::LIFTLEFT, 0, pulselength);
      servos.setPWM(servoID::LIFTRIGHT, 0, pulselength2);
      //servos.setPWM(servoID::CONTAINMENT,0,map(containmentAngle,0,180,SERVO_MIN,SERVO_MAX));
      delay(speed);
    }
  }
  currentArmAngle = angle;
}

void scoop(int angle)
{
  pulselength = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  servos.setPWM(servoID::SCOOP, 0, pulselength);
  delay(1500);
}

void gate(int angle)
{
  pulselength = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  servos.setPWM(servoID::GATE, 0, pulselength);
}

ISR(TIMER0_COMPA_vect)
{
  drivetime++;
  sampletime++;
}

void forwardBack(int direction, int time, int speed)
{
  double iy = 0;
  double ip = 0;
  double ir = 0;
  speed = map(speed, 0, 1000, 0, 255);
  int motorPowerRight = speed;
  int motorPowerLeft  = speed;
  int stopTime = 0;
  double dt = 0.001;
  double output = 0;
  double error = 0;
  bool started = false;
  bool finished = false;

  //Set Initial Motor Speeds
  analogWrite(MOTOR_RIGHT_EN, motorPowerRight);
  analogWrite(MOTOR_LEFT_EN, motorPowerLeft);   


  // Start Timer interrupts
  TCCR0A |= (1 << WGM01);
  TCCR0B |= (1 << CS01) | (1 << CS00);   // Set Prescalar to 64' (CLK_64)
  OCR0A = 0b11111010;      // Set output compare value to 250 for TOP every 1ms
  TIMSK0 |= (1 << OCIE0A); // Enable interrupts for Timer1 OCR2A

  while (true){
    //1.Start Streaming in IMU Data
    myIMU.getYawPitchRoll(y,p,r);
    if (sampletime > 20 && started)
    {
      recordYaw(y,p,r,motorPowerRight,motorPowerLeft);
      sampletime = 0;
    }

    if (((drivetime > stopTime) && (stopTime != 0))  || drivetime > 8000)
    {
      break;
    }
    
    if ((drivetime > IMU_START_DELAY) && !finished){
      if (!started)
      {
        //2.Start Recording Data
        started = true;
        iy = y;
        ip = p;
        ir = r;
        Serial.println("");
        Serial.print("KP is:");
        Serial.println(yaw_Kp,4);
        Serial.println("Starting Values:");
        Serial.println("Yaw,Pitch,Roll");
        Serial.print(iy,4);
        Serial.print(",");
        Serial.print(ip,4);
        Serial.print(",");
        Serial.print(ir,4);
        Serial.println("");
      }

      if (((drivetime > IMU_START_DELAY) && (drivetime < time + IMU_START_DELAY)))
      {
        //Feedback controller goes here
        error = iy - y;
        if (error < -180.0)
        {
            error += 360.0;
        }
        else if (error > 180.0)
        {
            error -= 360.0;
        }

        if (fabs(error) > 0.3)
        {
          output = yaw_Kp * error;
          if (output > 0)
          {
            if (direction == 1)
            {
              motorPowerRight = (int)(speed * (1.0 + (fabs(output)/100.0)));
              if (motorPowerRight > 255){
                motorPowerRight = 255;
              }
            }
            else if (direction == -1){
              motorPowerLeft = (int)(speed * (1.0 + (fabs(output)/100.0)));
              if (motorPowerLeft > 255){
                motorPowerLeft = 255;
              }
            }

          }
          else if (output < 0)
          {
            if (direction == 1)
            {
              motorPowerLeft = (int)(speed * (1.0 + (fabs(output)/100.0)));
              if (motorPowerLeft > 255){
                motorPowerLeft = 255;
              }
            }
            else if (direction == -1){
              motorPowerRight = (int)(speed * (1.0 + (fabs(output)/100.0)));
              if (motorPowerRight > 255){
                motorPowerRight = 255;
              }
            }
          }
        }

        setMotor(direction,motorPowerRight,MOTOR_RIGHT_EN,MOTOR_RIGHT_IN1,MOTOR_RIGHT_IN2);
        setMotor(direction,motorPowerLeft,MOTOR_LEFT_EN,MOTOR_LEFT_IN1,MOTOR_LEFT_IN2);
      }
      else
      {
        digitalWrite(MOTOR_RIGHT_IN1, LOW);
        digitalWrite(MOTOR_RIGHT_IN2, LOW);
        digitalWrite(MOTOR_LEFT_IN1, LOW);
        digitalWrite(MOTOR_LEFT_IN2, LOW);
        stopTime = drivetime + IMU_END_DELAY;
        finished = true;
      }
    }
  }

  iy = y;
  ip = p;
  ir = r;

  Serial.println("Ending Values:");
  Serial.println("Yaw,Pitch,Roll");
  Serial.print(iy,4);
  Serial.print(",");
  Serial.print(ip,4);
  Serial.print(",");
  Serial.print(ir,4);
  Serial.print(",");
  Serial.println("");
  Serial.println("Ending Motor PWM Values:");
  Serial.print("Right:");
  Serial.print(motorPowerRight);
  Serial.print("Left:");
  Serial.print(motorPowerLeft);
  Serial.println("");

  TIMSK0 &= ~(1 << OCIE0A); // Stop Timer Interrupts
  drivetime = 0;
}

void rotate(int direction, double angle,int speed)
{
  double iy = 0;
  double ip = 0;
  double ir = 0;
  double delta = 0;
  bool started = false;
  int rotationDirection = 0;
  int stopTime = 0;
  speed = map(speed, 0, 1000, 0, 255); // Map the potentiometer value from 0 to 255
  analogWrite(MOTOR_RIGHT_EN, speed);  // Send PWM signal to L298N Enable pin
  analogWrite(MOTOR_LEFT_EN, speed);   // Send PWM signal to L298N Enable pin
  // Start Timer interrupts
  TCCR0A |= (1 << WGM01);
  TCCR0B |= (1 << CS01) | (1 << CS00);   // Set Prescalar to 64' (CLK_64)
  OCR0A = 0b11111010;      // Set output compare value to 250 for TOP every 1ms
  TIMSK0 |= (1 << OCIE0A); // Enable interrupts for Timer0 OCR0

  if (direction == -1)
  {
    angle = angle*-1;
  }

  bool finished = false;
  while (true){
    //1.Start Streaming in IMU Data
    myIMU.getYawPitchRoll(y,p,r);
    if (sampletime > 20 && started)
    {
      recordYaw(y,p,r,speed,speed);
      sampletime = 0;
    }

    if (((drivetime > stopTime) && (stopTime != 0))  || drivetime > 8000)
    {
      digitalWrite(MOTOR_RIGHT_IN1, LOW);
      digitalWrite(MOTOR_RIGHT_IN2, LOW);
      digitalWrite(MOTOR_LEFT_IN1, LOW);
      digitalWrite(MOTOR_LEFT_IN2, LOW);
      break;
    }
    
    if ((drivetime > IMU_START_DELAY) && !finished){
      if (!started)
      {
        //2.Start Recording Data
        started = true;
        iy = y;
        ip = p;
        ir = r;
        Serial.println("");
        Serial.println("Starting Values:");
        Serial.println("Yaw,Pitch,Roll");
        Serial.print(iy,4);
        Serial.print(",");
        Serial.print(ip,4);
        Serial.print(",");
        Serial.print(ir,4);
        Serial.println("");

        //3.Calculate Angle to turn to.
        delta = iy + angle;
        if (delta < -180.0)
        {
            delta += 360.0;
        }
        else if (delta > 180.0)
        {
            delta -= 360.0;
        }

        //4.Direction to Rotate
        if (delta > 0)
        {
          rotationDirection = 1;
        }
        else
        {
          rotationDirection = -1;
        }
        Serial.print("Target Angle:");
        Serial.println(delta,4);
        Serial.println("");
      }

      //5.Start Rotation until hit tolerance.
      if (fabs(y - delta) >= ROTATION_TOLERANCE)
      {
        if (rotationDirection == 1)
        {
          digitalWrite(MOTOR_RIGHT_IN1, LOW);
          digitalWrite(MOTOR_RIGHT_IN2, HIGH);

          digitalWrite(MOTOR_LEFT_IN1, LOW);
          digitalWrite(MOTOR_LEFT_IN2, HIGH);
        }
        else if (rotationDirection == -1)
        {
          digitalWrite(MOTOR_RIGHT_IN1, HIGH);
          digitalWrite(MOTOR_RIGHT_IN2, LOW);

          digitalWrite(MOTOR_LEFT_IN1, HIGH);
          digitalWrite(MOTOR_LEFT_IN2, LOW);
        }
      }
      else
      {
        digitalWrite(MOTOR_RIGHT_IN1, LOW);
        digitalWrite(MOTOR_RIGHT_IN2, LOW);
        digitalWrite(MOTOR_LEFT_IN1, LOW);
        digitalWrite(MOTOR_LEFT_IN2, LOW);
        stopTime = drivetime + IMU_END_DELAY;
        finished = true;
      }
    }
  }

  iy = y;
  ip = p;
  ir = r;
  Serial.println("Ending Values:");
  Serial.println("Yaw,Pitch,Roll");
  Serial.print(iy,4);
  Serial.print(",");
  Serial.print(ip,4);
  Serial.print(",");
  Serial.print(ir,4);
  Serial.println("");

  TIMSK0 &= ~(1 << OCIE0A); // Stop Timer Interrupts
  drivetime = 0;
}

void recordYaw(double& yaw, double& pitch, double& roll,int right,int left)
{
  if (IMUDataCount < IMU_DATA_SIZE)
  {
    IMUData[IMUDataCount].time = drivetime - IMU_START_DELAY;
    IMUData[IMUDataCount].yaw = yaw ;
    IMUData[IMUDataCount].pitch = pitch;
    IMUData[IMUDataCount].roll = roll;
    IMUData[IMUDataCount].motorRight = right;
    IMUData[IMUDataCount].motorLeft = left;
    IMUDataCount++;
  }
  else
  {
    //Zero out array to prevent overflow.
    for (int i = 0 ; i < IMUDataCount;i++)
    {
      IMUData[i].time = 0;
      IMUData[i].yaw = 0;
      IMUData[i].pitch = 0;
      IMUData[i].roll = 0;
      IMUData[i].motorRight = 0;
      IMUData[i].motorLeft = 0;
    }
    IMUDataCount = 0;
  }
}

void outputYaw()
{
  Serial.println("");
  delay(500);
  Serial.println("");
  delay(500);
  Serial.println("Time,Yaw,Pitch,Roll,Right,Left");
  delay(1000);
  for (int i = 0;i<IMUDataCount;i++)
  {
    Serial.print(IMUData[i].time);
    Serial.print(",");
    Serial.print(IMUData[i].yaw,3);
    Serial.print(",");
    Serial.print(IMUData[i].pitch,3);
    Serial.print(",");
    Serial.print(IMUData[i].roll,3);
    Serial.print(",");
    Serial.print(IMUData[i].motorRight);
    Serial.print(",");
    Serial.print(IMUData[i].motorLeft);
    Serial.println("");
    delay(100);
  }
}

//EEPROM Management:
void dumpMemory(uint16_t stop){
  Serial.println("");
  Serial.print("DUMPING EEPROM to Address:");
  Serial.println(stop);
  int value = 0;
  if (stop > EEPROM.length()){
    stop = EEPROM.length();
  }
  for (uint16_t i = 0; i< stop;i++){
    value = EEPROM.read(i);
    Serial.print("Location:");
    Serial.print(i);
    Serial.print(" Value:");
    Serial.print(value);
    Serial.println("");
  }
}

void move(int direction, int time , int speed){
  pos_Right_i = 0;
  long posPrev_Right = 0;
  double velocity_Right = 0;
  long prevT_Right = micros();

  pos_Left_i = 0;
  long posPrev_Left = 0;
  double velocity_Left = 0;
  long prevT_Left = micros();
  double vrFilt = 0;
  double vrPrev = 0;
  double vlFilt = 0;
  double vlPrev = 0;
  double e_r = 0;
  double e_l = 0;
  float eintegral_r  = 0.0;
  float eintegral_l  = 0.0;
  double kpr = (double)kp_r;
  double kir = (double)ki_r;
  double kpl = (double)kp_l;
  double kil = (double)ki_l;
  double u_r = 0;
  double u_l = 0;
  int pwr_r = 0;
  int dir_r = 0;
  int pwr_l = 0;
  int dir_l = 0;
  double vtr = 0;
  double vtl = 0;
  double vr = 0;
  double vl = 0;
  double velocity_right = 0;
  double velocity_left = 0;
  double deltaT_Right = 0;
  double deltaT_Left = 0;
  long pos_Right = 0;
  long pos_Left = 0;
  long currT = micros();
  long startTime = millis();
  long currentTime = millis();
  long samplePrev = millis();
  while ((currentTime - startTime) < 1000)
  {
    //1.Start Streaming in IMU Data
    myIMU.getYawPitchRoll(y,p,r);
    currentTime = millis();
  }
  startTime = millis();
  while ((currentTime - startTime) <  time){

    myIMU.getYawPitchRoll(y,p,r);
    if ((currentTime - samplePrev) > 20)
    {
      recordYaw(y,p,r,(int) vrFilt, (int) vlFilt);
      samplePrev = millis();
    }

    currentTime = millis();
    // read the position in an atomic block
    // to avoid potential misreads
    pos_Right = 0;
    pos_Left = 0;
    //1.Read Position Values Safely.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      pos_Right = pos_Right_i;
      pos_Left = pos_Left_i;
    }

    //2. Compute Motor Velocities
    currT = micros();

    deltaT_Right = ( (double)  (currT-prevT_Right)) / 1.0e6;

    deltaT_Left  = ( (double) (currT-prevT_Left))   / 1.0e6;

    velocity_right = ((double)(pos_Right - posPrev_Right))  / deltaT_Right;
    velocity_left  = ((double)(pos_Left - posPrev_Left))   / deltaT_Left;

    posPrev_Right = pos_Right;
    posPrev_Left  = pos_Left;

    prevT_Right = currT;
    prevT_Left  = currT;

    //3. Convert count/s to RPM
    vr = velocity_right/700.0*60.0;
    vl = velocity_left/700.0*60.0;

    //4.Low-pass filter (25 Hz cutoff)
    vrFilt = 0.854*vrFilt + 0.0728*vr + 0.0728*vrPrev;
    vrPrev = vr;

    vlFilt = 0.854*vlFilt + 0.0728*vl + 0.0728*vlPrev;
    vlPrev = vl;


    //5. Set a target speed for both motors.
    if (direction == 1)
    {
      vtr = (double)(speed);
      vtl = (double)(-1*speed);
    }
    else if (direction == -1)
    {
      vtr = (double)(-1*speed);
      vtl = (double)(speed);
    }



    // // //6. Run PID Algorithm.
    // motorRight.evalu(vrFilt,vtr,deltaT_Right,pwr_r,dir_r);
    // motorLeft.evalu(vlFilt,vtl,deltaT_Left,pwr_l,dir_l);


    e_r = vtr - vrFilt;
    eintegral_r += e_r*(deltaT_Right);
    u_r = kpr*e_r + kir*eintegral_r ;
    dir_r = 1;
    if (u_r<0){
      dir_r = -1;
    }
    pwr_r = (int) fabs(u_r);
    if(pwr_r > 255){
      pwr_r = 255;
    }

    e_l =  vtl - vlFilt;
    eintegral_l += e_l*(deltaT_Left);
    u_l = kpl*e_l + kil*eintegral_l;
    dir_l = 1;
    if (u_l<0){
      dir_l = -1;
    }
    pwr_l = (int) fabs(u_l);
    if(pwr_l > 255){
      pwr_l = 255;
    }
    //PLANT PROCESS
    setMotor(dir_r,pwr_r,pwm[0],in1[0],in2[0]);  
    setMotor(dir_l,pwr_l,pwm[1],in1[1],in2[1]);

    //Debug.
    // Serial.print(eintegral_r,3);
    // Serial.print(",");
    // Serial.print(eintegral_l,3);
    // Serial.print(vrFilt,3);
    // Serial.print(",");
    // Serial.print(vlFilt,3);
    // Serial.print(",");
    // Serial.print(vtr,3);
    // Serial.print(",");
    // Serial.print(vtl,3);
    // // Serial.print(",");
    // // // Serial.print(u_r,4);
    // // Serial.print(",");
    // // Serial.print(e_r,4);
    // // Serial.print(",");
    // // Serial.print(e_l,4);
    // Serial.println();
  }
  setMotor(0,0,0,in1[0],in2[0]);
  setMotor(0,0,0,in1[1],in2[1]);
}

// void rotate(int direction, double angle,int speed)
// {

// }

void init_motor()
{
  for(int k = 0; k < 2; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoderRight,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoderLeft,RISING);
  Serial.println("Motors Initialized!");
}
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  analogWrite(pwm,pwmVal);
  if(dir == 1)
  {
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else if(dir == -1)
  {
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else
  {
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}
void readEncoderRight(){
  int b = digitalRead(encb[0]);
  if(b > 0){
    pos_Right_i++;
  }
  else{
    pos_Right_i--;
  }
}
void readEncoderLeft(){
  int b = digitalRead(encb[1]);
  if(b > 0){
    pos_Left_i++;
  }
  else{
    pos_Left_i--;
  }
}