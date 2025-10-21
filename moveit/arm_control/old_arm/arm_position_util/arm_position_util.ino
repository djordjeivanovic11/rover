#include <AccelStepper.h>
#include <MultiStepper.h>

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>


#define NODE_NAME "robotteensy"
#define NAMESPACE ""
#define SUBSCRIPTION_NAME "arm_target_motor_positions"
#define PUBLISHER_NAME "get_arm_position"

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t subscriber;
rcl_publisher_t publisher;

std_msgs__msg__Int32MultiArray msg;
std_msgs__msg__Int32MultiArray out_msg;

MultiStepper armSteppers;
// pins: step, direction (ENA = 12 globally)
#define GLOBAL_ENABLE 12
AccelStepper azimuthStepper(AccelStepper::DRIVER, 25, 24);
AccelStepper shoulderStepper(AccelStepper::DRIVER, 3, 2);
AccelStepper elbowStepper(AccelStepper::DRIVER, 11, 10);
AccelStepper wristLowStepper(AccelStepper::DRIVER, 5, 4);
AccelStepper wristHighStepper(AccelStepper::DRIVER, 9, 8);
AccelStepper gripperStepper(AccelStepper::DRIVER, 7, 6);

void subscription_callback(const void * msgin) {
  // const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  if (msg.data.size >= 6) {
    long targets[6];
    for (int i = 0; i < 6; ++i) {
      targets[i] = msg.data.data[i];
    }
    armSteppers.moveTo(targets);
    //tone(13, 4, 1000);
  }
}

int speed_multiplier = 2;

void setup() {
  azimuthStepper.setMaxSpeed(2500*speed_multiplier);
  azimuthStepper.setMinPulseWidth(5);
  armSteppers.addStepper(azimuthStepper);

  shoulderStepper.setMaxSpeed(2500*speed_multiplier);
  shoulderStepper.setMinPulseWidth(5);
  armSteppers.addStepper(shoulderStepper);

  elbowStepper.setMaxSpeed(2500*speed_multiplier);
  elbowStepper.setMinPulseWidth(5);
  armSteppers.addStepper(elbowStepper);

  wristLowStepper.setMaxSpeed(2500*speed_multiplier);
  wristLowStepper.setMinPulseWidth(5);
  armSteppers.addStepper(wristLowStepper);

  wristHighStepper.setMaxSpeed(16000*speed_multiplier);
  wristHighStepper.setMinPulseWidth(5);
  armSteppers.addStepper(wristHighStepper);

  gripperStepper.setMaxSpeed(2000*speed_multiplier);
  gripperStepper.setMinPulseWidth(5);
  armSteppers.addStepper(gripperStepper);

  pinMode(GLOBAL_ENABLE, OUTPUT);
  digitalWrite(GLOBAL_ENABLE, HIGH);

  // Configure LED pin
  pinMode(13, OUTPUT);
  tone(13, 1, 3000);
  // digitalWrite(32, HIGH);
  // while (true) {
  //   digitalWrite(31, HIGH);
  // }

  set_microros_transports();

  allocator = rcl_get_default_allocator();

  // create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, NODE_NAME, NAMESPACE, &support);

  // subscriber

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    SUBSCRIPTION_NAME
  );

  // INIT_INT32MULTIARRAY(msg, 10, 10)
  msg.data.capacity = 10;
  msg.data.size = 0;
  msg.data.data = (int32_t*)malloc(msg.data.capacity * sizeof(int32_t));

  msg.layout.dim.capacity = 10;
  msg.layout.dim.size = 0;
  msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*)malloc(msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

  // publisher

  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    PUBLISHER_NAME
  );

  out_msg.data.capacity = 10;
  out_msg.data.size = 0;
  out_msg.data.data = (int32_t*)malloc(out_msg.data.capacity * sizeof(int32_t));

  out_msg.layout.dim.capacity = 10;
  out_msg.layout.dim.size = 0;
  out_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*)malloc(out_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

}

void loop() {
  // delay(100);
  // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));
  digitalWrite(GLOBAL_ENABLE, LOW);
  if (armSteppers.run()) {
    tone(13, 2);
  } else {
    noTone(13);
    digitalWrite(13, LOW);
  }

  // prepare publish message
  out_msg.data.data[0] = azimuthStepper.currentPosition();
  out_msg.data.data[1] = shoulderStepper.currentPosition();
  out_msg.data.data[2] = elbowStepper.currentPosition();
  out_msg.data.data[3] = wristLowStepper.currentPosition();
  out_msg.data.data[4] = wristHighStepper.currentPosition();
  out_msg.data.data[5] = gripperStepper.currentPosition();
  out_msg.data.size = 6;

  rcl_ret_t rc = rcl_publish(&publisher, &out_msg, NULL);
  //digitalWrite(13, LOW);
  if (rc != RCL_RET_OK) {
    // ggs it messed up and i dont know what to do if it fails here
    // maybe have an error LED?
  }

}
