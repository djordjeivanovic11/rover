#include "MicroROS.h"

namespace uROS {

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#include <std_msgs/msg/int32.h>

void Init_MicroROS(const char* node_name, const char* node_namespace) {
    set_microros_transports();

    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, node_name, node_namespace, &support);
}

void Publisher::Init() {
    rclc_publisher_init_default(
        &_publisher,
        &node,
        _msg->GetTypeSupport(),
        _name
    );
}

bool Publisher::Publish() {
    // _out_msg.data = data;
    rcl_ret_t rc = rcl_publish(&_publisher, _msg->GetRawMsg(), NULL);
    return rc == RCL_RET_OK;
}


}