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
#include "./subsystems/conveyor.h"
#include "./subsystems/deposit.h"
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

std_msgs::String ianOutputMsg;
ros::Publisher ianOutputPub("/listener/ian_output", &ianOutputMsg);

// std_msgs::Float32 localizerAngle;
// ros::Publisher localizerAnglePub("/jetson/localizer_angle", &localizerAngle);

std_msgs::Float32MultiArray poseStep;
ros::Publisher poseStepPub("/jetson/pose_step", &poseStep);

std_msgs::Float32 conveyorSpeed;
ros::Publisher conveyorSpeedPub("/jetson/conveyor_speed", &conveyorSpeed);

std_msgs::Bool plungeTop;
ros::Publisher plungeTopPub("/jetson/plunge_top", &plungeTop);

std_msgs::Bool plungeBot;
ros::Publisher plungeBotPub("/jetson/plunge_bot", &plungeBot);

Drivetrain drivetrain;
// Localizer localizer;
Conveyor conveyor;
Deposit deposit;

int driveSpeed = 0;
bool drivetrainEnable = false;
bool drivetrainAngle = false;
// bool localizerEnable = false;

float poseStepVals[3] = { 0.0, 0.0, 0.0};

int odomIterator = 0;

float icc = 0.0;

static unsigned long previousDriveMillis = 0;
static unsigned long previousConveyorMillis = 0;
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

void conveyorCurrentCallback(const std_msgs::Int32 &conveyorCurrent) {
  conveyor.setConveyorCurrent(conveyorCurrent.data);
}

void conveyorPlungeCallback(const std_msgs::Int32 &plungeSpeed){
  conveyor.setPlungeSpeed(plungeSpeed.data);
}

void depositOpenCallback(const std_msgs::Bool &depositOpenMsg) {
  deposit.setOpen(depositOpenMsg.data);
}

ros::Subscriber<std_msgs::Float32> driveSpeedSub("/drivetrain/drive", &drivetrainSpeedCallback);
ros::Subscriber<std_msgs::Int32> driveStateSub("/drivetrain/state", &drivetrainSwitchStateCallback);
ros::Subscriber<std_msgs::Float32> driveICCSub("/drivetrain/icc", &drivetrainICCallback);
ros::Subscriber<std_msgs::Int32> conveyorSub("/digger/conveyor_current", &conveyorCurrentCallback);
ros::Subscriber<std_msgs::Int32> plungeSub("/digger/plunge", &conveyorPlungeCallback);
ros::Subscriber<std_msgs::Bool> depositOpen("/deposit/open", &depositOpenCallback);


// ros::Subscriber<std_msgs::Float32> localizerErrorSub("/localizer/error", &localizerErrorCallback);
// ros::Subscriber<std_msgs::Bool> localizerEnableSub("/localizer/enable", &localizerEnableCallback);


void setup()
{
  Serial.begin(115200);  // Set baud rate to 115200
  nh.getHardware()->setBaud(115200);  // Tell rosserial to use the same baud rate
  nh.initNode();

  // do{
  //   nh.initNode();
  //   delay(500); // Delay to prevent flooding with connection attempts
  // }while (!nh.connected()); 

  nh.advertise(ianOutputPub);
  // nh.advertise(localizerAnglePub);
  nh.advertise(poseStepPub);
  nh.advertise(conveyorSpeedPub);
  nh.advertise(plungeTopPub);
  nh.advertise(plungeBotPub);

  nh.subscribe(driveSpeedSub);
  // nh.subscribe(localizerErrorSub);
  // nh.subscribe(localizerEnableSub);
  nh.subscribe(driveStateSub);
  nh.subscribe(driveICCSub);
  nh.subscribe(conveyorSub);
  nh.subscribe(plungeSub);
  nh.subscribe(depositOpen);

  drivetrain.init();
  // localizer.init();
  conveyor.init();
  deposit.init();

  SPI.begin();
  Wire.begin();
  Wire.setClock(800000L);
}

void loop()
{
  currentMillis = millis();

  // Conveyor gets looped every X milliseconds
  if (currentMillis - previousConveyorMillis >= CONVEYOR_INTERVAL) {
    previousConveyorMillis = currentMillis;
    
    conveyor.loop();
    
    plungeBot.data = conveyor.isAtBot();
    plungeBotPub.publish(&plungeBot);

    plungeTop.data = conveyor.isAtTop();
    plungeTopPub.publish(&plungeTop);

    conveyorSpeed.data = conveyor.getConveyorSpeed();
    conveyorSpeedPub.publish(&conveyorSpeed);

    deposit.loop();
  }


  // Drivetrain gets looped every X milliseconds
  if (currentMillis - previousDriveMillis >= DRIVETRAIN_INTERVAL) {

    previousDriveMillis = currentMillis;

    nh.spinOnce();

    drivetrain.loop();

    // localizer.loop();

    // Prints for Drivetrain Speed
    // String ianOutputString = String(drivetrain.getDriveSpeed());
    // ianOutputMsg.data = ianOutputString.c_str();
    // ianOutputPub.publish(&ianOutputMsg);

    // update Odom
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