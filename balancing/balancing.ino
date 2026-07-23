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
const float alpha = 0.02;


bool FirstTime = true;
ArduPID myController;
float setpoint = 0.0;


//current angle calculation requires slices of time.
unsigned long previousTime;
unsigned long currentTime;
float dt;


//pins for motor driver
const byte ENA = 19; //yellow
const byte IN1 = 18; //blue
const byte IN2 = 17; //green
const byte ENB = 25; //orange
const byte IN3 = 26; //purple
const byte IN4 = 27; //gold


L298N motor1(ENA, IN1, IN2);
L298N motor2(ENB, IN3, IN4);


float input;
float pidout;


float kp = 2.0;
float ki = 0.0;
float kd = 0.0;


void setup() {
  Serial.begin(115200);
  //delay(10000);


  Serial.println("ESP32 started");
  Wire.begin(23, 22);  //Wire.begin(sda, scl)
  Serial.println("Wire started");


  Wire.beginTransmission(0x68);
  Wire.write(0x6B);      // PWR_MGMT_1 register
  Wire.write(0x00);      // Wake up device
  Wire.endTransmission();
  delay(100);


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
  myController.setOutputLimits(-150, 150);
  motor1.stop();
  motor2.stop();
 
}
/*
void PrintDebug(float accelAngle, float gyroRate, float filteredAngle, float deltaTime)
{


  Serial.print("Accel: ");
  Serial.print(accelAngle, 2);


  Serial.print(" | Gyro: ");
  Serial.print(gyroRate, 2);


  Serial.print(" deg/s | Filtered: ");
  Serial.print(filteredAngle, 2);


  Serial.print(" deg | dt: ");
  Serial.print(deltaTime * 1000.0, 2);
  Serial.println(" ms");
}
*/


void loop() {
  // put your main code here,to run repeatedly:
  //time
  currentTime = micros();
  dt= (currentTime-previousTime)/1000000.0;
  previousTime = currentTime;
 
  float currentangle;
  ReadIMU();  
  float gyrorate = Gy - 3.0;
  Accelangle = CalculateAccelAngle(Ax, Az);


  if (FirstTime){
     previousangle = Accelangle;
    FirstTime = false;
  }
  currentangle = alpha * Accelangle + (1 - alpha) * (previousangle + gyrorate * dt);
  //PrintDebug(Accelangle, gyrorate, currentangle, dt);
  previousangle = currentangle;
  input = currentangle;
  pidout = myController.compute(input);


  Serial.print("Angle: ");
  Serial.print(currentangle);
  Serial.print(" PID: ");
  Serial.print(pidout);


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




float CalculateAccelAngle(float ax, float az){
    return atan2(ax,az) * 180.0 /PI;
}




void MotorControl(float output){
  /*
  if (abs(output) < 2){
    motor1.stop();
    motor2.stop();
    return;
  }*/


  float motorSpeed = 60.0 + (abs(output) / 150.0) * (150.0 - 60.0);
  motorSpeed = constrain(motorSpeed, 60, 150);
  motor1.setSpeed(motorSpeed);
  motor2.setSpeed(motorSpeed);
  //Serial.print("   Motor PWM: "); Serial.println(motorSpeed); debugging
 
  if(output < 0){
     motor1.forward();
     motor2.forward();
  }
  else if (output > 0){
    motor1.backward();
    motor2.backward();
  }
  else{
    motor1.stop();
    motor2.stop();
  }
}

void updatePIDConstants(){
    if (!Serial.available()) return;

    String command = Serial.readStringUntil('\n');
    command.trim();
        if (command.startsWith("KP:")) {

            kp = command.substring(3).toFloat();

        }
        else if (command.startsWith("KI:")){
          ki = command.substring(3).toFloat();
        }
        else if (command.startsWith("KD:")){
          kd = command.substring(3).toFloat();
        }
       else{
          Serial.println("Invalid command.");
    }
    myController.setTunings(kp, ki, kd);
    Serial.print("PID constants updated: ");
    Serial.print("KP: ");
    Serial.print(kp);
    Serial.print('\n');
    Serial.print("KI: ");
    Serial.print(ki);
    Serial.print('\n');
    Serial.print("KD: ");
    Serial.print(kd );
    Serial.print('\n');
    }










