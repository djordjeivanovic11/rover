#include "MicroROS.h"
#include <Arduino.h>

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

// #include <std_msgs/msg/int32_multi_array.h>

#define NODE_NAME "drive"
#define NAMESPACE ""
// #define SUBSCRIPTION_NAME "arm_target_motor_positions"
// #define PUBLISHER_NAME "get_arm_position"

// rclc_executor_t executor;
// rcl_subscription_t subscriber;
// rcl_publisher_t publisher;

struct IntMsg: public uROS::Msg {
  std_msgs__msg__Int32 _out_msg;
  const rosidl_message_type_support_t* GetTypeSupport() {
    return ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, std_msgs, msg, Int32)();
  }
  void Init() { _out_msg.data = 0; };
  void* GetRawMsg() {
    return &_out_msg;
  }
};

struct FloatMsg: public uROS::Msg {
  std_msgs__msg__Float32 _out_msg;
  const rosidl_message_type_support_t* GetTypeSupport() {
    return ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, std_msgs, msg, Float32)();
  }
  void Init() { _out_msg.data = 0.0; };
  void* GetRawMsg() {
    return &_out_msg;
  }
};

IntMsg testMsg;
uROS::Publisher p("drive_test", &testMsg);

FloatMsg testFloat;
uROS::Publisher p2("drive_float", &testFloat);

void setup() {

  // Configure LED pin
  pinMode(13, OUTPUT);
  tone(13, 1, 3000);

  uROS::Init_MicroROS(NODE_NAME, NAMESPACE);

  p.Init();
  p2.Init();
}

int i = 0;
float d = 0;
std_msgs__msg__Int32 _out_msg;
void loop() {
  delay(1000);

  // _out_msg.data = i++;
  testMsg._out_msg.data = i++;
  p.Publish();

  testFloat._out_msg.data = d;
  d += 0.1;
  p2.Publish();
  // p.PublishRaw(&_out_msg);
  // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
