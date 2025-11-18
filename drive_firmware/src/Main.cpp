#include <Arduino.h>
#include <VescUart.h>

#include "MicroROS.h"
#include "Messages.h"

// #include <urc_msgs/msg/tank_drive_target.h>

#define NODE_NAME "drive"
#define NAMESPACE "drive"

#define MAX_RPM 15000

VescUart VESC;

// struct TankDriveTargetMsg: uROS::Msg {
//   urc_msgs__msg__TankDriveTarget _msg;
//   const rosidl_message_type_support_t* GetTypeSupport() {
//     return ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, urc_msgs, msg, TankDriveTarget)();
//   }
//   void Init() {
//     _msg.left_speed = 0.0;
//     _msg.right_speed = 0.0;
//   }
//   void Set(float left, float right) {
//     _msg.left_speed = left;
//     _msg.right_speed = right;
//   }
//   void* GetRawMsg() {
//     return &_msg;
//   }
// };

// TankDriveTargetMsg subMsg;
// uROS::Subscriber Sub("drive_target", subMsg, [](const void * msgin) {
//   tone(13, 1, 1000 * subMsg._msg.data);
//   i = subMsg._msg.data;
//   logMsg.Set("Got sub!");
//   LogPub.Publish();
// });
// uROS::Subscriber Sub("drive_target", subMsg);

bool update = false;

IntMsg leftDriveMsg;
uROS::Subscriber LeftDriveSub("left_rpm", leftDriveMsg);

IntMsg rightDriveMsg;
uROS::Subscriber RightDriveSub("right_rpm", rightDriveMsg);

void setup() {
  // Configure LED pin
  pinMode(13, OUTPUT);
  tone(13, 1, 3000);

  uROS::Init_MicroROS(NODE_NAME, NAMESPACE);

  leftDriveMsg.Init();
  rightDriveMsg.Init();

  LeftDriveSub.Init();
  RightDriveSub.Init();

  Serial1.begin(115200, SERIAL_8N1);
  VESC.setSerialPort(&Serial1);
}

unsigned long lastUpdate = 0;

void loop() {
  uROS::Spin(0);

  float leftRPM = leftDriveMsg.GetValue();
  float rightRPM = rightDriveMsg.GetValue();
  if (leftRPM < -MAX_RPM) {
    leftRPM = -MAX_RPM;
  } else if (leftRPM > MAX_RPM) {
    leftRPM = MAX_RPM;
  }
  if (rightRPM < -MAX_RPM) {
    rightRPM = -MAX_RPM;
  } else if (rightRPM > MAX_RPM) {
    rightRPM = MAX_RPM;
  }

  VESC.setRPM(leftRPM * 1.0, 0);
  VESC.setRPM(leftRPM * 1.0, 2);
  VESC.setRPM(rightRPM * 1.0, 3);
  VESC.setRPM(rightRPM * 1.0, 4);
}


// #include "MicroROS.h"
// #include <Arduino.h>

// // #include <rcl/rcl.h>
// // #include <rclc/rclc.h>
// // #include <rclc/executor.h>
// #include <std_msgs/msg/int32.h>

// // #include <std_msgs/msg/int32_multi_array.h>

// #define NODE_NAME "drive"
// #define NAMESPACE ""
// // #define SUBSCRIPTION_NAME "arm_target_motor_positions"
// // #define PUBLISHER_NAME "get_arm_position"

// // rclc_executor_t executor;
// // rcl_subscription_t subscriber;
// // rcl_publisher_t publisher;

// uROS::Publisher p;

// void setup() {

//   // Configure LED pin
//   pinMode(13, OUTPUT);
//   tone(13, 1, 3000);

//   uROS::Init_MicroROS(NODE_NAME, NAMESPACE);

//   p.Init("drive_test");
// }

// int i = 0;
// std_msgs__msg__Int32 _out_msg;
// void loop() {
//   delay(1000);

//   _out_msg.data = i++;
//   p.PublishRaw(&_out_msg);
//   // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//   // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
// }
