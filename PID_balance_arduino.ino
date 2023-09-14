#include <Wire.h>
#include <Servo.h>

Servo right_prop; //3
Servo left_prop;  //6
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 
float Acceleration_angle[2]; float Gyro_angle[2]; float Total_angle[2];
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
float kp=2.75;//3.55
float ki=0.003;//0.003
float kd=1.2;//2.05
///////////////////////////////////////////////

double throttle=1300;       //initial value of throttle to the motors
float desired_angle = 0; 

void setup() {
  Wire.begin();            //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(19200);
  
  right_prop.attach(3); 
  left_prop.attach(6);  
  
  time = millis(); 
  left_prop.writeMicroseconds(1000); 
  right_prop.writeMicroseconds(1000);
  delay(7000); 
}

void loop() {
    if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input.length() > 3) {
      
      int colonIndex1 = input.indexOf(',');
      int colonIndex2 = input.indexOf(',', colonIndex1 + 1);
      if (colonIndex1 >= 0 && colonIndex2 >= 0) {
        kp = input.substring(0, colonIndex1).toFloat();
        ki = input.substring(colonIndex1 + 1, colonIndex2).toFloat();
        kd = input.substring(colonIndex2 + 1).toFloat();
      }
    }
  }
    timePrev = time;  
    time = millis();  
    elapsedTime = (time - timePrev) / 1000; 
    
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); 
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
    Acc_rawX=Wire.read()<<8|Wire.read(); 
    Acc_rawY=Wire.read()<<8|Wire.read();
    Acc_rawZ=Wire.read()<<8|Wire.read();

     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    
    Wire.beginTransmission(0x68);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true); 
   
    Gyr_rawX=Wire.read()<<8|Wire.read(); 
    Gyr_rawY=Wire.read()<<8|Wire.read();
      Gyro_angle[0] = Gyr_rawX/131.0; 
      Gyro_angle[1] = Gyr_rawY/131.0 - 1.1;

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
//PID
error = Total_angle[0] - desired_angle;
  
    pid_p = kp*error;                                    // pid_p calculation

  if(-5 <error < 5)
  {
    pid_i = pid_i+(ki*error);                            //pid_i calculation
  }

    pid_d = kd*((error - previous_error)/elapsedTime);   //pid_d calculation

        PID = pid_p + pid_i + pid_d;

if(PID < -200)
{
  pid_i=-200;
}
if(PID > 200)
{
  pid_i=200;
}

Serial.print(kp);
Serial.print(",");

Serial.print(ki);
Serial.print(",");

Serial.print(kd);
Serial.print(",");

Serial.print(Total_angle[0]);
Serial.print(",");

Serial.print(PID);
Serial.println(",");

pwmLeft = throttle - PID;
pwmRight = throttle + PID;

if(pwmRight < 1100)
{
  pwmRight= 1100;
}
if(pwmRight > 1300)
{
  pwmRight=1300;
}

if(pwmLeft < 1100)
{
  pwmLeft= 1100;
}
if(pwmLeft > 1300)
{
  pwmLeft=1300;
}

left_prop.writeMicroseconds(pwmLeft);
right_prop.writeMicroseconds(pwmRight);
previous_error = error; 
}