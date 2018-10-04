
/* Written by Abrar Almjally
 * Controls a simple robot using the Adafruit v2 motor sheild. 
 * Robot controlled vai a HC-06 Bluetooth chip.
 * refrence https://forum.makeblock.com/t/how-to-make-a-bluetooth-remote-control-system-for-makeblock-tank/4186 
 */

 #include "MeMCore.h"
#include <SoftwareSerial.h>
#include <Wire.h>

/*SENSORE DEFINITIONS*/
MeDCMotor leftMotor(M1); // Motor Connector to the left 
MeDCMotor rightMotor(M2);// Motor Connector  to the right 
MeBluetooth bluetooth(PORT_5);

//Variable definitions
MeBuzzer buzzer;
int8_t motorSpeed = 60;
//int M_time = 2190;// 3.4v
int M_time = 2100;// 
float min_voltage = 3.3;
int8_t startmotorSpeed =80;
int8_t leftSpeed = 60; //left motor speed
int8_t rightSpeed = 60; //right motor speed
double LturnSpeed = 0.6; //factor for reduced speed of inner belt during curves
double turnSpeed = 0.5; //factor for reduced speed of inner belt during curve
int8_t turn_motorSpeed = 120; //right motor speed
/*

-Minimum-start-speed: 60 (reliable minimum speed)
-Minimum-rolling-speed: Determine the minimal required power at which the robot keeps moving.
30 (drive 60 for 0.5 seconds and then reduce speed to 30)
-Power-speed-ratio Find out for each power setting (range 0-255) the actual speed in cm per second.
-Acceleration: Determine time to accelerate from zero to a certain speed*/


//Gyro variable
MeGyro gyro;
float heading = 0;
float Heading ;
float head_X, head_Y , head_Z ;
int interval = 100; //interval for ultrasonic sweeping // the minimal measure interval is 100 milliseconds
unsigned long prev,prevFW; //previous time

// Set up some variables. 
char state = 'n';
String SPring = "start ";


void setup() {
  // Will be used to read and write to app via Bluetooth.
  Serial.begin(115200); 
  bluetooth.begin(115200);
 //Serial.print("Bluetooth Start! the turn working, the Forward, and gyro ");
   float voltage = analogRead(A0) / 1024.0 * 10.0;
   if (voltage < min_voltage)
   {
    buzz();
   }
Serial.print(voltage);
 gyro.begin();
  delay(1667);
  prev = millis();
}

// Loops all time arduino is running.
void loop() {

  if ((millis() - prev) > interval) //if 100msec time is over  measure the heading
  {
    prev = millis();
    gyro.update();
    Heading = gyro.getAngleZ();
  } 
  
  if(bluetooth.available()) {
    // If can read from Bluetooth read.
    state =  readBT();        //Read Bluetooth input
    
    if(state == 'l'){
      turn(82, 'l', turn_motorSpeed); //call function turn with 84 degrees, right direction and 120 motorspeed
      heading = 0;
      state = 'n';  
      delay(5000);     
    } else if (state == 'f'){
      forward();
      state = 'n';  
      delay(M_time); 
      Stop();    
    } else if (state == 'r'){
      // TurnRight(); //from source code //https://github.com/Makeblock-official/mBot/blob/master/mBot-default-program/mBot-default-program.ino
      turn(82, 'r', turn_motorSpeed); //call function turn with 84 degrees, right direction and 120 motorspeed
      heading = 0;
      state = 'n';  
      delay(5000);     
    } else if (state == 'b'){
      buzz();
      state = 'n';     
    } else if (state == 'e'){
      // 'e' signals end of the instructions so confirmation is sent back.
      bluetooth.print("x");//it is working but the massege that is showing is not correct /thats is not quite right 
      state = 'n';
      Stop();
      delay(2000);
    } 
  } 
  
 //  Serial.println(SPring); // the robot  is working but it is stocu in the if 
  
}




 //Make robot buzz.
void buzz() {
  buzzer.tone(1200, 700);  //Buzzer sounds 1200Hz for 1000ms
  Stop();
  delay(2000);              //Pause for 2000ms, Buzzer no sound

}

/*BT read function*/
char readBT(){
  char btIn;
  btIn = (char) bluetooth.read();
  return btIn;
}
/* move 29 cm */
void forward() {
  leftMotor.run(-motorSpeed);
  rightMotor.run(motorSpeed+11.2);
  //stop();
}


void Backward() {
  leftMotor.run(motorSpeed);
  rightMotor.run(-motorSpeed);
}

void turn_right(){
  leftMotor.run(0);
  rightMotor.run(-rightSpeed+5);
}


void turn_left() {
  /*leftMotor.run(-leftSpeed*LturnSpeed);
  rightMotor.run(rightSpeed);*/
  leftMotor.run(leftSpeed-3);
  rightMotor.run(0);
  
}


//******* FUNCTION turn left or right for specific angle controlled by a GYRO-sensor ***
void turn(int angle, char rightorleft, uint8_t velocity)
{
    gyro.begin(); //reset gyro
    heading = 0;
  while (abs(heading) < angle)
  {
    
    if (rightorleft == 'r')
    {
      leftMotor.run(-velocity); /* value: between -255 and 255. */
      rightMotor.run(-velocity); /* value: between -255 and 255. */
      
    }
    else if (rightorleft == 'l')
    {
     // a left turn when the right motor receives more power then the left motor (L<R). 
      leftMotor.run(velocity); /* value: between -255 and 255. */
      rightMotor.run(velocity); /* value: between -255 and 255. */
    }
    gyro.update();
    heading = fmod(gyro.getAngleZ(),360.0); /*The values returned by gyro.getAngleX/Y/Z, after gyro.update() */
    /*are given in +/- degrees deviation from what has been measured/calculated at gyro.begin(), when X, Y, and Z are set to “0”. So, if Z = 3.0, it means that the vehicle has turned 3 degrees to the right (I prefer right to be positive), since the last calibration of the gyro.
    the heading (Z-axis) is defined as 360 deg - that values for pitch and roll (X and Y axes) are defined within a range of +/-90 deg*/ 
      }
  //stop
Stop();
}

void Stop() {
  leftMotor.stop();
  rightMotor.stop();
}
