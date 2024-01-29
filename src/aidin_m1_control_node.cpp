#include <iostream>
#include <cmath>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std;

class JointControl: public rclcpp::Node
{
public:
    JointControl() : Node("aidin_m1_control_node")
    {
        // Subscribe to JointPos_sim and JointVel_sim topics
        sub_jointpos = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/JointPos_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                joint1_pos = msg->data[0];
                joint2_pos = msg->data[1];
                joint3_pos = msg->data[2];
                joint4_pos = msg->data[3];
                joint5_pos = msg->data[4];
                joint6_pos = msg->data[5];
                joint7_pos = msg->data[6];
                joint8_pos = msg->data[7];
                joint9_pos = msg->data[8];
                joint10_pos = msg->data[9];
                joint11_pos = msg->data[10];
                joint12_pos = msg->data[11];
                CalculateAndPublishTorque();
            });

        sub_jointvel = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/JointVel_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                joint1_vel = msg->data[0];
                joint2_vel = msg->data[1];
                joint3_vel = msg->data[2];
                joint4_vel = msg->data[3];
                joint5_vel = msg->data[4];
                joint6_vel = msg->data[5];
                joint7_vel = msg->data[6];
                joint8_vel = msg->data[7];
                joint9_vel = msg->data[8];
                joint10_vel = msg->data[9];
                joint11_vel = msg->data[10];
                joint12_vel = msg->data[11];
                CalculateAndPublishTorque();
            });

        sub_simtime = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", rclcpp::QoS(10).best_effort(),
            [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
                sim_time = msg->clock.sec + (msg->clock.nanosec)/1000000000.0;
                CalculateAndPublishTorque();
            });

        pub_torque = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Torque_sim", 10);

        // sub_gains = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        //     "/aidin_m1/Gains_sim", 10,
        //     [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        //         kp[0] = msg->data[0];
        //         kp[1] = msg->data[1];
        //         kp[2] = msg->data[2];
        //         kd[0] = msg->data[3];
        //         kd[1] = msg->data[4];
        //         kd[2] = msg->data[5];
        //     });

        sub_angles = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Angles_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                ag[0] = msg->data[0];
                ag[1] = msg->data[1];
            });


        // Publish desired joint poses
        pub_desiredpos = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/DesiredPos", 10);
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointpos;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointvel;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_simtime;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_torque;
    // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_gains;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_angles;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_desiredpos;

    // Joint angle
    float joint1_pos, joint2_pos, joint3_pos, joint4_pos, joint5_pos, joint6_pos, joint7_pos, joint8_pos, joint9_pos, joint10_pos, joint11_pos, joint12_pos;
    // Joint velocity
    float joint1_vel, joint2_vel, joint3_vel, joint4_vel, joint5_vel, joint6_vel, joint7_vel, joint8_vel, joint9_vel, joint10_vel, joint11_vel, joint12_vel;
    double sim_time;                    // Gazebo simulation time
    double ag[3];                       // Angles


    void CalculateAndPublishTorque()
    {
        std_msgs::msg::Float32MultiArray torque_msg;
        torque_msg.data.clear();

        torque_msg.data.push_back(ag[0]);
        torque_msg.data.push_back(0);
        torque_msg.data.push_back(0);
        torque_msg.data.push_back(-ag[0]);
        torque_msg.data.push_back(0);
        torque_msg.data.push_back(0);
        torque_msg.data.push_back(ag[0]);
        torque_msg.data.push_back(0);
        torque_msg.data.push_back(0);
        torque_msg.data.push_back(-ag[0]);
        torque_msg.data.push_back(0);
        torque_msg.data.push_back(0);

        pub_torque->publish(torque_msg);


        std_msgs::msg::Float32MultiArray desiredpos_msg;
        desiredpos_msg.data.clear();

        desiredpos_msg.data.push_back(ag[0]);
        desiredpos_msg.data.push_back(0);
        desiredpos_msg.data.push_back(0);
        desiredpos_msg.data.push_back(ag[0]);
        desiredpos_msg.data.push_back(0);
        desiredpos_msg.data.push_back(0);
        desiredpos_msg.data.push_back(ag[0]);
        desiredpos_msg.data.push_back(0);
        desiredpos_msg.data.push_back(0);
        desiredpos_msg.data.push_back(ag[0]);
        desiredpos_msg.data.push_back(0);
        desiredpos_msg.data.push_back(0);

        pub_desiredpos->publish(desiredpos_msg);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointControl>());
    rclcpp::shutdown();

    return 0;
}
