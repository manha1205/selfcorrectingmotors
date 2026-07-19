#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
float Ax, Ay, Az;
float Gx, Gy, Gz;

bool FirstTime = true;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    Wire.begin(21, 22);
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
    ReadIMU();
    CalculateAccelAngle();
    if (firstTime){
      previousangle = Accelangle;
      firstTime = false;
    }
    currentangle = 0.02*Accelangle + 0.98 * (previousangle + gyrorate *dt);
}

void ReadIMU(){
  sensors_event_t a, g, temp; 
  mpu.getEvent(&a, &g, &temp);

  Ax = a.acceleration.x;
  Ay = a.acceleration.y;
  Az = a.acceleration.z

  Gx = g.gyro.x;
  Gy= g.gyro.y;
  Gz=g.gyro.z;
  // Serial.print("Acceleration X: ");
  // Serial.print(Ax);
  // Serial.print(", Y: ");
  // Serial.print(Ay);
  // Serial.print(", Z: ");
  // Serial.print(Az);
  // Serial.print("Rate x: ");
  // Serial.print(g.gyro.x); //only for debugging

}

