#include <Wire.h>
#include <Arduino.h>
#include "./robotMap.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "./subsystems/drivetrain.h"
// #include "./subsystems/localizer.h"
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

std_msgs::String ianOutputMsg;
ros::Publisher ianOutputPub("/listener/ian_output", &ianOutputMsg);

// std_msgs::Float32 localizerAngle;
// ros::Publisher localizerAnglePub("/jetson/localizer_angle", &localizerAngle);

std_msgs::Float32MultiArray poseStep;
ros::Publisher poseStepPub("/jetson/pose_step", &poseStep);

Drivetrain drivetrain;
// Localizer localizer;

int driveSpeed = 0;
bool drivetrainEnable = false;
bool drivetrainAngle = false;
// bool localizerEnable = false;

float poseStepVals[3] = { 0.0, 0.0, 0.0};

int odomIterator = 0;

float icc = 0.0;

static unsigned long previousMillis = 0;
unsigned long currentMillis = millis();

void drivetrainSpeedCallback(const std_msgs::Float32 &driveSpeedMsg) {
  float driveSpeed = driveSpeedMsg.data;
  drivetrain.setDriveSpeed(driveSpeed);
}

void drivetrainSwitchStateCallback(const std_msgs::Int32 &driveStateMsg) {
  int drivetrainState = driveStateMsg.data;
  drivetrain.setState(drivetrainState);
}


void drivetrainICCallback(const std_msgs::Float32 &driveICCMsg) {
  float icc = driveICCMsg.data;
  drivetrain.setYICC(icc);
}

// void localizerErrorCallback(const std_msgs::Float32 &localizerErrorMsg) {
//   localizer.setError(localizerErrorMsg.data);
// }

// void localizerEnableCallback(const std_msgs::Bool &localizerEnableMsg) {
//   localizerEnable = localizerEnableMsg.data;

//   if (localizerEnable == true) {
//     localizer.enable();
//   } else {
//     localizer.disable();
//   }
// }

ros::Subscriber<std_msgs::Float32> driveSpeedSub("/drivetrain/drive", &drivetrainSpeedCallback);
ros::Subscriber<std_msgs::Int32> driveStateSub("/drivetrain/state", &drivetrainSwitchStateCallback);
ros::Subscriber<std_msgs::Float32> driveICCSub("/drivetrain/icc", &drivetrainICCallback);
// ros::Subscriber<std_msgs::Float32> localizerErrorSub("/localizer/error", &localizerErrorCallback);
// ros::Subscriber<std_msgs::Bool> localizerEnableSub("/localizer/enable", &localizerEnableCallback);


void setup()
{
  nh.initNode();
  nh.advertise(ianOutputPub);
  // nh.advertise(localizerAnglePub);
  nh.advertise(poseStepPub);
  nh.subscribe(driveSpeedSub);
  // nh.subscribe(localizerErrorSub);
  // nh.subscribe(localizerEnableSub);
  nh.subscribe(driveStateSub);
  nh.subscribe(driveICCSub);

  drivetrain.init();
  // localizer.init();

  SPI.begin();
  Wire.begin();
  Wire.setClock(800000L);
}

void loop()
{
  currentMillis = millis();

  // Drivetrain gets looped every 10 milliseconds
  if (currentMillis - previousMillis >= DRIVETRAIN_INTERVAL) {

    nh.spinOnce();

    previousMillis = currentMillis;

    drivetrain.loop();

    // localizer.loop();

    // if (drivetrain.isEnabled()) {
    String ianOutputString = String("Back Left Speed: ") + String(drivetrain.getSpeed(1)) + String("         Front Left Speed: ") + String(drivetrain.getSpeed(0));
    ianOutputMsg.data = ianOutputString.c_str();
    ianOutputPub.publish(&ianOutputMsg);
    // }

    // Regardless of whether the localizer is enabled, return the correct angle
    // localizerAngle.data = localizer.getAngle();
    // localizerAnglePub.publish(&localizerAngle);


    //update Odom
    odomIterator ++;
    if(odomIterator >= ODOM_FREQUENCY){

      drivetrain.stepOdom();

      poseStepVals[0] = drivetrain.getPoseStepX();
      poseStepVals[1] = drivetrain.getPoseStepY();
      poseStepVals[2] = drivetrain.getPoseStepTheta();

      std_msgs::Float32MultiArray stepMsg;
      stepMsg.data_length = 3;
      stepMsg.data = poseStepVals;
      poseStepPub.publish(&stepMsg);

      odomIterator = 0;
    }
  }
}