#pragma once

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <type_traits>

namespace uROS {
    void Init_MicroROS(const char* node_name, const char* node_namespace);

    struct Msg {
        const virtual rosidl_message_type_support_t* GetTypeSupport();
        virtual void Init();
        virtual void* GetRawMsg();
    };

    class Publisher {
    public:
        template<typename MsgType>
        Publisher(const char* name, MsgType* msg) {
            _name = name;
            static_assert(std::is_convertible<MsgType, Msg>::value, "");
            _msg = msg;
        };
        void Init();
        bool Publish();

    private:
        const char* _name;
        rcl_publisher_t _publisher;
        Msg* _msg;
        // std_msgs__msg__Int32 _out_msg;
    };

}
// #endif