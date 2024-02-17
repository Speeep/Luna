#include <Wire.h>
#include <Arduino.h>
#include "./robotMap.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "./subsystems/drivetrain.h"
#include "./subsystems/localizer.h"
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

std_msgs::String ianOutputMsg;
ros::Publisher ianOutputPub("/listener/ian_output", &ianOutputMsg);

std_msgs::Float32 localizerAngle;
ros::Publisher localizerAnglePub("/jetson/localizer_angle", &localizerAngle);

std_msgs::Float32MultiArray poseStep;
ros::Publisher poseStepPub("/jetson/pose_step", &poseStep);

Drivetrain drivetrain;
Localizer localizer;

int driveSpeed = 0;
bool drivetrainEnable = false;
bool drivetrainAngle = false;
bool localizerEnable = false;

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

// TODO: Delete after testing ICCs
void drivetrainICCallback(const std_msgs::Float32 &driveICCMsg) {
  float iccStep = driveICCMsg.data;
  float newIcc = icc + iccStep;
  icc = newIcc;
}

void localizerErrorCallback(const std_msgs::Float32 &localizerErrorMsg) {
  localizer.setError(localizerErrorMsg.data);
}

void localizerEnableCallback(const std_msgs::Bool &localizerEnableMsg) {
  localizerEnable = localizerEnableMsg.data;

  if (localizerEnable == true) {
    localizer.enable();
  } else {
    localizer.disable();
  }
}

ros::Subscriber<std_msgs::Float32> driveSpeedSub("/drivetrain/drive", &drivetrainSpeedCallback);
ros::Subscriber<std_msgs::Int32> driveStateSub("/drivetrain/state", &drivetrainSwitchStateCallback);
ros::Subscriber<std_msgs::Float32> driveICCSub("/drivetrain/icc_step", &drivetrainICCallback);
ros::Subscriber<std_msgs::Float32> localizerErrorSub("/localizer/error", &localizerErrorCallback);
ros::Subscriber<std_msgs::Bool> localizerEnableSub("/localizer/enable", &localizerEnableCallback);


void setup()
{
  Serial.begin(57600);
  SPI.begin();
  Wire.begin();
  Wire.setClock(800000L);

  nh.initNode();
  nh.advertise(ianOutputPub);
  nh.advertise(localizerAnglePub);
  nh.advertise(poseStepPub);
  nh.subscribe(driveSpeedSub);
  nh.subscribe(localizerErrorSub);
  nh.subscribe(localizerEnableSub);

  drivetrain.init();
  localizer.init();
}

void loop()
{
  nh.spinOnce();

  currentMillis = millis();

  // Drivetrain gets looped every 10 milliseconds
  if (currentMillis - previousMillis >= DRIVETRAIN_INTERVAL) {

    previousMillis = currentMillis;

    drivetrain.loop();

    localizer.loop();

    if (drivetrain.isEnabled()) {
      String drivetrainWheel0Speed = String(drivetrain.getRightWheelpodAngle());
      String ianOutputString = drivetrainWheel0Speed;
      ianOutputMsg.data = ianOutputString.c_str();
      ianOutputPub.publish(&ianOutputMsg);
    }

    // Regardless of whether the localizer is enabled, return the correct angle
    localizerAngle.data = localizer.getAngle();
    localizerAnglePub.publish(&localizerAngle);


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