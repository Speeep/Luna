#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Wire.h>

#!/usr/bin/env python

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

Wire wire;

std_msgs::Float32 Magnetic_pos_msg;
ros::Publisher Magnetic_pos_pub("magnetic_pos", &Magnetic_pos_msg);

//Magnetic sensor things
int magnetStatus = 0; //value of the status register (MD, ML, MH)
float startAngle = 53.00; //starting angle 
                          // NOTE: in future this will need to be an array for multiple encoders

// Kalman myFilter(0.05, 16, 1023, 0); // Kalman filter

//I2C things
int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8

float degAngle;
float radAngle;

// NOTE: in future this needs to be reworked for multiplexing
float ReadAngle()
{ 
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request

  // //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  int rawAngle = (highbyte | lowbyte) & 0x0fff; //extract last 12 bits

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = static_cast<float>(rawAngle) * 0.087890625 - startAngle;

  if(degAngle < 0){
    degAngle += 360;
  }

  if(degAngle > 180){
    degAngle -= 360;
  }

  radAngle = degAngle *(M_PI/180);

  return radAngle;
}


void checkMagnetPresence()
{  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    //Serial.print("Magnet status: ");
    //Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21)      
  }   
}

ros::Subscriber<std_msgs::_Float32> sub("magnetic_error", ReadAngle);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //start serial - tip: don't use serial if you don't need it (speed considerations)
  Wire.begin(); //start i2C  
	Wire.setClock(800000L); //fast clock
  checkMagnetPresence(); //check the magnet (blocks until magnet is found)
  ReadAngle();
  // startAngle = degAngle; //update startAngle with degAngle - for taring

  // pinMode(13, OUTPUT); // output on pin 14

  nh.initNode(); //make a reading so the degAngle gets updated
  nh.subscribe(sub);
  nh.advertise(Magnetic_pos_pub);  // Advertise the magnetic_pos topic

  // wire.attach(2); // attach it to pin 3
}

void loop() {
  // put your main code here, to run repeatedly:  
  float angle = ReadAngle(); //ask the value from the sensor
  
  // Publish the magnetic sensor data to the ROS topic
  Magnetic_pos_msg.data = angle;
  Magnetic_pos_pub.publish(&Magnetic_pos_msg);
  
  
  nh.spinOnce();
  delay(1);
}