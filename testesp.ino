#include "FastIMU.h"
#include <Wire.h>
#include "ArduPID.h"
#include <L298N.h>

MPU6500 IMU;
calData calibration = {0};
AccelData accelData;
GyroData gyroData;
float Ax, Ay, Az, Gx;
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
const int ENA;
const int IN1;
const int IN2;
const int ENB;
const int IN3;
const int IN4;


float input;
float pidout;

float kp = 1.0;
float ki = 0.0;
float kd = 0.0;

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
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
}
void loop() {
  // put your main code here,to run repeatedly:
  //time
  currentTime = micros();
  dt= (currentTime-previousTime)/1000000.0;
  previousTime = currentTime;
  
  float currentangle;
 
    ReadIMU();
    
  float gyrorate = Gx; 
    Accelangle = CalculateAccelAngle(Ay, Az);

    if (FirstTime){
       previousangle = Accelangle;
      FirstTime = false;
    }
    currentangle = 0.02*Accelangle + 0.98 * (previousangle + gyrorate *dt);
  
    previousangle = currentangle;
    input = currentangle;
    pidout = myController.compute(input);
    myController.debug();
    // MotorControl(pidout);

    
}

void ReadIMU(){
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);


   Ax = accelData.accelX;
   Ay = accelData.accelY;
   Az = accelData.accelZ;

   Gx= gyroData.gyroX;

  
}
float CalculateAccelAngle(float ay, float az){
    return atan2(ay,az) * 180.0 /PI;
}

void MotorControl(float output){
  float motorSpeed = abs(output);

  if( output > 0){
      // Motor A
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        // Motor B: 
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
  }
  else if (outout < 0){
    
  }

}
