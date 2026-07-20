#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "ArduPID.h"


Adafruit_MPU6050 mpu;
float Ax, Ay, Az;
float Gx, Gy, Gz;
float previousangle;
float Accelangle;

bool FirstTime = true;
ArduPID myController;
float setpoint = 0.0;

unsigned long previousTime;
unsigned long currentTime;
float dt;


float input;
float pidout;

float kp = 1.0;
float ki = 0.0;
float kd = 0.0;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    Wire.begin(21, 22);
    previousTime = micros();
    myController.setTunings(kp, ki, kd);

     if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");


}

void loop() {
  // put your main code here,to run repeatedly:
  //time
  currentTime = micros();
  dt= (currentTime-previousTime)/1000000.0;
  previousTime = currentTime;
  
  float currentangle;
  float gyrorate = Gx;
    ReadIMU();
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

    
}

void ReadIMU(){
  sensors_event_t a, g, temp; 
  mpu.getEvent(&a, &g, &temp);

   Ax = a.acceleration.x;
   Ay = a.acceleration.y;
   Az = a.acceleration.z;

   Gx= g.gyro.x;
  // Serial.print("Acceleration X: ");
  // Serial.print(Ax);
  // Serial.print(", Y: ");
  // Serial.print(Ay);
  // Serial.print(", Z: ");
  // Serial.print(Az);
  // Serial.print("Rate x: ");
  // Serial.print(g.gyro.x); //only for debugging
}
float CalculateAccelAngle(float ay, float az){
    return atan2(ay,az);
}

