#include "FastIMU.h"
#include <Wire.h>
#include "ArduPID.h"
#include <L298N.h>

MPU6500 IMU;
calData calibration = {0};
AccelData accelData;
GyroData gyroData;
float Ax, Ay, Az, Gy;
float previousangle;
float Accelangle;


bool FirstTime = true;
ArduPID myController;
float setpoint = 0.0;

//current angle calculation requires slices of time.
unsigned long previousTime;
unsigned long currentTime;
float dt;

//pins for motor driver
const int ENA =0;
const int IN1 = 1;
const int IN2 = 2;
const int ENB = 3;
const int IN3 = 4;
const int IN4= 5;

L298N motor1(ENA, IN1, IN2);
L298N motor2(ENB, IN3, IN4);

float input;
float pidout;

float kp = 1.0;
float ki = 0.25;
float kd = 0.25;

void setup() {
  Serial.begin(115200);
  delay(10000);

  Serial.println("ESP32 started");
  Wire.begin(21, 22);
  Serial.println("Wire started");

  int err = IMU.init(calibration);

  if (err!=0){
    Serial.println("IMU initialization failed");
    Serial.println(err);
    while (1) {
    delay(10000);
}
  }
 else{
  Serial.println("IMU initialization worked");

 }
 ReadIMU();
  previousTime = micros();
  myController.setTunings(kp, ki, kd);
  myController.setOutputLimits(-128, 128);
  motor1.stop();
  motor2.stop();
}



void loop() {
  // put your main code here,to run repeatedly:
  //time
  currentTime = micros();
  dt= (currentTime-previousTime)/1000000.0;
  previousTime = currentTime;
  
  float currentangle;
 
    ReadIMU();
    
  float gyrorate = Gy; 
    Accelangle = CalculateAccelAngle(Ax, Az);

    if (FirstTime){
       previousangle = Accelangle;
      FirstTime = false;
    }
    currentangle = 0.02*Accelangle + 0.98 * (previousangle + gyrorate *dt);
  
    previousangle = currentangle;
    input = currentangle;
    pidout = myController.compute(input);
    MotorControl(pidout);
}

void ReadIMU(){
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);


   Ax = accelData.accelX;
   Ay = accelData.accelY;
   Az = accelData.accelZ;

   Gy= gyroData.gyroY;

  
}
float CalculateAccelAngle(float ay, float az){
    return atan2(ay,az) * 180.0 /PI;
}
void MotorControl(float output){
  float motorSpeed = abs(output);
  motor1.setSpeed(motorSpeed);
  motor2.setSpeed(motorSpeed);

  if( output > 0){
     motor1.forward();
     motor2.forward();
  }
  else if (output < 0){
    motor1.backward();
    motor2.backward();
  }
  else{
    motor1.stop();
    motor2.stop();
  }
  

}
