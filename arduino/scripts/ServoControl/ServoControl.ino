#include <Wire.h>
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;

std_msgs::Float32 left_wheelpod_angle_msg;
ros::Publisher left_wheelpod_angle_pub("/drivetrain/left_wheelpod_angle", &left_wheelpod_angle_msg);

// BTS7960 motor driver constants
int R_IS = 6;
int R_EN = 2;
int R_PWM = 3;
int L_IS = 7;
int L_EN = 4;
int L_PWM = 5;

// Magnetic sensor things
float startAngleArray[] = {0.925025, 0, 0, 0, 0, 0, 0, 0}; // starting angle
float radAngle;

// I2C things
int lowbyte;   // raw angle 7:0
word highbyte; // raw angle 7:0 and 11:8
float angleSetpoint = 1;

float ReadAngle(uint8_t encoderNumber)
{
  // Multiplexer things
  Wire.beginTransmission(0x70);
  Wire.write(1 << encoderNumber);
  if (Wire.endTransmission() != 0)
  {
    // Handle I2C communication error
    Serial.println("Error setting multiplexer channel");
    return radAngle;  // Return the last known valid value
  }

  // Encoder things
  // 7:0 - low
  Wire.beginTransmission(0x36);
  Wire.write(0x0D);
  if (Wire.endTransmission() != 0)
  {
    Serial.println("Error starting transmission to the sensor");
    return radAngle;
  }

  Wire.requestFrom(0x36, 1);
  if (Wire.available() == 0)
  {
    Serial.println("Error receiving data from the sensor");
    return radAngle;
  }
  lowbyte = Wire.read();

  // 11:8 - high
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  if (Wire.endTransmission() != 0)
  {
    Serial.println("Error starting transmission to the sensor");
    return radAngle;
  }

  Wire.requestFrom(0x36, 1);
  if (Wire.available() == 0)
  {
    Serial.println("Error receiving data from the sensor");
    return radAngle;
  }
  highbyte = Wire.read();
  highbyte = highbyte << 8;

  int rawAngle = (highbyte | lowbyte) & 0x0fff;
  radAngle = static_cast<float>(rawAngle) * 0.001533981 - startAngleArray[encoderNumber];

  if (radAngle < 0)
  {
    radAngle += 6.28319;
  }
  if (radAngle > 3.14159)
  {
    radAngle -= 6.28319;
  }

  return radAngle;
}

void setMotorEffort(int effort)
{
  if (effort > 100)
  {
    effort = 100;
  }
  if (effort < -100)
  {
    effort = -100;
  }
  if (effort >= 0)
  {
    analogWrite(L_PWM, map(effort, 0, 100, 15, 100));
    analogWrite(R_PWM, 0);
  }
  if (effort < 0)
  {
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, map(abs(effort), 0, 100, 12, 100));
  }
}

void angleSetpointCallback(const std_msgs::Float32 &angle_msg)
{
  angleSetpoint = angle_msg.data;
}

ros::Subscriber<std_msgs::Float32> angleSetpointSub("/arduino/left_wheelpod_angle_setpoint", &angleSetpointCallback);

void setup()
{
  Serial.begin(57600);
  
  Wire.begin();           // start i2C
  Wire.setClock(800000L); // fast clock
  pinMode(R_IS, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_IS, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  digitalWrite(R_IS, HIGH);
  digitalWrite(L_IS, HIGH);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  nh.initNode();
  nh.advertise(left_wheelpod_angle_pub);
  nh.subscribe(angleSetpointSub);
}
void loop()
{
  nh.spinOnce();
  float angle = ReadAngle(1); // ask the value from the sensor

  // Publish the magnetic sensor data to the ROS topic
  left_wheelpod_angle_msg.data = angle;
  left_wheelpod_angle_pub.publish(&left_wheelpod_angle_msg);

  float p = 50;
  float error = angle - angleSetpoint;

  // normalize error within -pi to pi
  if (error > PI)
  {
    error -= 2 * PI;
  }
  if (error < -PI)
  {
    error += 2 * PI;
  }

  int effort = (int)(error * p);
  setMotorEffort(effort);
  delay(100);
}
