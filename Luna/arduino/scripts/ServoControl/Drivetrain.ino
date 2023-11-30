#include <SPI.h>
#include <mcp2515.h>
#include <TimerOne.h>
#include <Kalman.h>
#include <MsTimer2.h>
#include <FastPID.h>

#ifdef PARAM_SHIFT
#undef PARAM_SHIFT
#define PARAM_SHIFT 4
#endif

// ******************************* Parameter Configuration ******************************* //
#define pos_interval 10 // 10ms position loop calculation interval
#define interval 2000   // 2ms sampling interval in microseconds

// Position loop PID parameters. After implementing the velocity loop, usually only the P term is needed.
const float pos_Kp = 0.05;  // 0.15  
const int pos_Ki = 0;         
const int pos_Kd = 0;

// Velocity loop PID parameters
// Increase Ki based on load, fine-tune Kp
float speed_Kp = 18;  //
float speed_Ki = 10;  //
float speed_Kd = 0.4; //<0.5

MCP2515 mcp2515(53);        // CS(SS) to pin 53 on MEGA2560
Kalman myFilter(0.05, 16, 1023, 0); // Kalman filter

// ******************************* End ******************************* //

struct can_frame canMsgIn;  // Received CAN frame
struct can_frame canMsgOut; // CAN frame to be sent
int angle[2] = {0};                 // Suppose 2 motors totally
int angle_last[2] = {0};
int RPM[2] = {0};
int actualCurrent[2] = {0};
int T[2] = {0};

int RAW_RPM; // Raw RPM value
int delta_pos[2] = {0}; // Rotor position difference between two samples
char recv[50]; // String received from the serial port
long sum_delta_pos[2] = {0};     // Sum of position differences  
long set_pos[2] = {0};
int set_speed[2] = {0};
int set_current[2] = {0};
long position[2] = {0}; // Current rotor position
const float i1 = 136.53333 / (1000000 / interval); // Intermediate constant for integral velocity conversion

const long speed_Hz = 1000000 / interval;
FastPID speed_PID_0(speed_Kp, speed_Ki, speed_Kd, speed_Hz, 15, true);
FastPID speed_PID_1(speed_Kp, speed_Ki, speed_Kd, speed_Hz, 15, true);

const long pos_Hz = 1000 / pos_interval;
FastPID pos_PID_0(pos_Kp, pos_Ki, pos_Kd, pos_Hz, 15, true);
FastPID pos_PID_1(pos_Kp, pos_Ki, pos_Kd, pos_Hz, 15, true);

int TAG; // Debug variable

int calc_Position(const long setPos, const int motorID)
{
  return constrain(
      pos_Kp * (setPos - position[motorID]) + 0 * sum_delta_pos[motorID],
      -16384, 16384);
}

// int calc_Speed(const long setSpeed, const int motorID)
// {
//   //TODO: May need to be divided into different speeds
//   return speed_PID.step(setSpeed, RPM[motorID]);
// }

void handler_PID_Position()
{
  printMessage(canMsgIn.can_id - 0x201); // Output information
  // set_speed[0] =  pos_PID_0.step(set_pos[0], position[0]);
  set_speed[0] = calc_Position(set_pos[0], 0);
} 

void handler_PID_Speed()
{
  set_current[0] = speed_PID_0.step(set_speed[0], RPM[0]);
  set_current[1] = speed_PID_1.step(set_speed[1], RPM[1]);
  setMotorCurrent();
}

void initMotor()
{
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Timer1.initialize(interval);
  Timer1.attachInterrupt(checkCANmsg);
  MsTimer2::set(pos_interval, handler_PID_Position);
  MsTimer2::start();
}

// Update motor parameters, perform velocity loop PID calculation, send CAN frame, highest response time
void updateInfo(const int motorID)
{

  angle[motorID] = toRealData(canMsgIn.data[0], canMsgIn.data[1]);
  RAW_RPM = toRealData(canMsgIn.data[2], canMsgIn.data[3]);
  // Kalman filter
  RPM[motorID] = (int)myFilter.getFilteredValue(RAW_RPM);
  // No filter
  // RPM[motorID] = RAW_RPM;
  actualCurrent[motorID] = toRealData(canMsgIn.data[4], canMsgIn.data[5]);
  T[motorID] = canMsgIn.data[6];

  sum_delta_pos[motorID] += set_pos[motorID] - position[motorID];

  delta_pos[motorID] = angle[motorID] - angle_last[motorID];
  if (abs(RPM[motorID]) < 1200) // Low-speed state
  {
    // Full rotation
    if (delta_pos[motorID] < -8000)
      position[motorID]++;
    else if (delta_pos[motorID] > 8000)
      position[motorID]--;
    // Not a full rotation
    else
      position[motorID] += delta_pos[motorID];
  }
  else if (abs(RPM[motorID]) < 6000) // Medium-speed state
  {
    if (RPM[motorID] < 0 && delta_pos[motorID] > 4000)
    {
      position[motorID] -= 8192;
    }
    else if (-delta_pos[motorID] > 4000 && RPM[motorID] > 0)
    {
      position[motorID] += 8192;
    }
  }
  else // High-speed state, directly approximate integral of speed
    position[motorID] += (RPM[motorID] * i1) * 8192;

  angle_last[motorID] = angle[motorID];
  handler_PID_Speed();
  //  set_current[0] = calc_Speed(set_speed[motorID], motorID);
  //   setMotorCurrent();
}

void setMotorCurrent()
{ // Set motor current  -16384 ~ 16384 ----> -20A ~ +20A
  canMsgOut.can_id = 0x200;
  canMsgOut.can_dlc = 8;
  canMsgOut.data[0] = (char)(set_current[0] / 256);     // High 8 bit
  canMsgOut.data[1] = (char)(set_current[0] % 256); // Low 8 bit
  canMsgOut.data[2] = (char)(set_current[1] / 256);     //High 8 bit
  canMsgOut.data[3] = (char)(set_current[1] % 256); // Low 8 bit
  mcp2515.sendMessage(&canMsgOut);
}

void printMessage(int motorID)
{
  // Serial.print("ID: ");
  // Serial.print("<<<<#########");
  // Serial.print(delta_pos[motorID]);
  // Serial.print("\t");

  Serial.print(long(position[motorID]));
  Serial.print("\t");
  // Serial.print(0.001 * pos_Kp *(set_pos[0] - position[motorID]));
  // Serial.print("\t");

  // Serial.print(RAW_RPM);
  // Serial.print("\t");

  // Serial.print("Angle: ");
  // Serial.print(RPM[motorID]);
  // Serial.print("\t");
  Serial.print(set_speed[0]);
  Serial.print("\t");

  // // Serial.print("RPM: ");
  Serial.print(set_pos[0]);
  Serial.print("\t");

  // Serial.print("I: ");
  // Serial.print(actualCurrent[motorID]);
  // Serial.print("\t");

  // Serial.print("T: ");
  // Serial.print(T[motorID]);
  // Serial.print("\t");

  // Serial.print("PID: ");
  // Serial.print(set_speed[0]);
  Serial.print("\t");

  //  Serial.print(set_current[0]+set_speed[0]);
  // Serial.print("\t");
  // Serial.print("\t#########>>>>");
  Serial.println();
}

inline int toRealData(unsigned char DataH, unsigned char DataL)
{
  return int(word(DataH, DataL));
}

inline bool CANmsgComing()
{
  return mcp2515.readMessage(&canMsgIn) == MCP2515::ERROR_OK;
}

void checkCANmsg()
{
  if (CANmsgComing())
  {
    updateInfo(canMsgIn.can_id - 0x201);
  }
}

void setup()
{
  Serial.begin(115200);
  initMotor();

  // If PID parameters are incorrect
  if (speed_PID_0.err())
  {
    Serial.println("There is a configuration error!");
    for (;;);
  }
}

void loop()
{
  // Receive messages from the serial port
  int length;
  if (Serial.available() > 0)
  {
    length = Serial.readBytes(recv, 50);
    recv[length] = '\0';
    set_pos[0] = atol(recv);   // Use the received number as the position set value
  }
  // printMessage(canMsgIn.can_id - 0x201); // Output information
  set_pos[0] += 8192;
  delay(400);
}
