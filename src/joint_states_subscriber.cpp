#include <functional>
#include <memory>

#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ecat_data_buffer.h"
#include "pid_algorithm.h"

bool dataOk = false;
int i = 0;
int target_p[6] = {0, 0, 0, 0, 0, 0};
double code2rad = 0.00000299605617523193;
C_PID motor(200.0f, 0.5f, 0.0f, 500.0f, 5.0f, 2.0f, 0.1f); // now
C_PID motor_v(200.0f, 0.4f, 0.0f, 500.0f, 5.0f, 2.0f, 0.1f);
C_PID motor_p(150.0f, 0.3f, 0.0f, 500.0f, 5.0f, 2.0f, 0.1f);
// C_PID(float Kp, float Ki, float Kd, float UMax, float UiMax, float UdMax, float ts)

//(float Kp, float Ki, float Kd, float UMax, float UiMax, float UdMax, float ts)

// C_PID motor;  // 使用默认构造函数创建对象

class JointStatesPublisherSubscriber : public rclcpp::Node
{
public:
    JointStatesPublisherSubscriber()
        : Node("joint_states_publisher_subscriber")
    {
        // Publisher for /joint_states topic
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        std::cout << "asdf" << std::endl;

        // Subscriber for /effort_controller/commands topic
        // effort_command_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        //     "/effort_controller/commands",
        //     10,
        //     std::bind(&JointStatesPublisherSubscriber::effort_command_callback, this, std::placeholders::_1));

        // Initialize JointState message
        joint_state_msg_.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        joint_state_msg_.position.resize(6);
        joint_state_msg_.velocity.resize(6);
        joint_state_msg_.effort.resize(6);
        std::cout << "asdf 2" << std::endl;
        // Set a timer to publish joint states periodically
        this->create_wall_timer(std::chrono::milliseconds(2),
                                std::bind(&JointStatesPublisherSubscriber::publish_joint_states, this));
    }

private:
    void publish_joint_states()
    {
        std::cout << "asd 3 f" << std::endl;
        memset(&feedback, 0, sizeof(feedback));
        dataOk = edb_pull_fdbk(&feedback);

        if (dataOk)
        {
            for (i = 0; i < joint_state_msg_.name.size(); i++)
            {
                joint_state_msg_.position[i] = feedback.motor_fdbk[i + P_START].feedbk_postion;
                joint_state_msg_.effort[i] = feedback.motor_fdbk[i + P_START].feedbk_torque;
                joint_state_msg_.velocity[i] = feedback.motor_fdbk[i + P_START].feedbk_speed; // 从底层获得电机的位置、速度、力矩反馈
            }
        }

        // Publish JointState message
        joint_state_publisher_->publish(joint_state_msg_);

        memset(&new_ref, 0, sizeof(new_ref));

        int tau_ff_5 = 40;
        // int tau_ff_4 = 50;
        int FF;
        std::cout << "start" << std::endl;

        // new_ref.motor_ref[5 + P_START].target_torque =  -(joint_state_msg_.position[5] - target_p[5])* code2rad * 100 + tau_ff_5;
        // new_ref.motor_ref[4 + P_START].target_torque =  -(joint_state_msg_.position[4] - target_p[4])* code2rad * 150 + tau_ff_4;
        // new_ref.motor_ref[4 + P_START].target_torque = 0;
        std::cout << "asdf 4" << std::endl;

        motor.fpFB = joint_state_msg_.position[5] * code2rad;
        motor.CalPID();
        // motor_v.Des = motor_p.fpU;
        if (joint_state_msg_.velocity[5] > 0)
        {
            FF = tau_ff_5;
        }
        else
        {
            FF = -tau_ff_5;
        }

        new_ref.motor_ref[5 + P_START].target_torque = motor.fpU + FF; // 命令
        // new_ref.motor_ref[5 + P_START].target_torque = 100 ;

        std::cout << new_ref.motor_ref[5 + P_START].target_torque << std::endl;

        // joint_state_msg_.position[5]
        // new_ref.motor_ref[5 + P_START].target_torque = 100;
        // new_ref.motor_ref[4 + P_START].target_torque = -100;
        dataOk = edb_push_ref(&new_ref);
    }

    // void effort_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    // {
    //     // Process Float64MultiArray message from /effort_controller/commands topic
    //     RCLCPP_INFO(get_logger(), "Received command with %ld elements", msg->data.size());
    //     for (size_t i = 0; i < msg->data.size(); ++i)
    //     {
    //         RCLCPP_INFO(get_logger(), "  Command[%ld]: %.2f", i, msg->data[i]);
    //         // Implement logic to handle commands (e.g., control joints)
    //         // Example: control_joints(i, msg->data[i]);
    //     }
    // }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_command_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    GROUP_REFERENCE new_ref;
    GROUP_FEEDBACK feedback;
    sensor_msgs::msg::JointState joint_state_msg_;
};

int main(int argc, char *argv[])
{
    int shmid = shmget(SHM_KEY, SHM_SIZE, 0666);
    if (shmid == -1)
    {
        perror("shmget");
        return EXIT_FAILURE;
    }

    void *appPtr;

    appPtr = shmat(shmid, 0, 0);
    if (appPtr == (void *)-1)
    {
        return -1;
    }

    edb_init(appPtr, SHM_SIZE, false);

    motor.fpUMax = 500;
    motor.fpDes = 0 * code2rad;
    // motor.fpEMin = 2;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatesPublisherSubscriber>());
    rclcpp::shutdown();
    return 0;
}