#ifndef _LEG_PLUGIN_HH_
#define _LEG_PLUGIN_HH_

#include "rclcpp/rclcpp.hpp"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <deque>
#include <numeric>
#include <vector>

namespace gazebo
{
    class aidin_m1_plugin : public ModelPlugin
    {
    public:
        aidin_m1_plugin();
        ~aidin_m1_plugin();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void OnUpdate();

    private:
        void ROSCallbackTorque_sim(const std_msgs::msg::Float32MultiArray::ConstSharedPtr torque);
        ignition::math::Vector3d ComputeCenterOfMass();
        void UpdateContactHistory(std::deque<bool> &history, bool contact_detected);
        double CalculateWeightedContact(const std::deque<bool> &history, const std::vector<double> &weights);
        void UpdateDistanceHistory(std::deque<double> &history, double distance);
        double CalculateAverageDistance(const std::deque<double> &history);

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_torque;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bodypose;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointpos;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointvel;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bodypos;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bodyvel;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_imu;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_contact;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_distance;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_link_force;  // for force


        physics::ModelPtr model;
        physics::JointControllerPtr joint1_Controller_;
        physics::JointControllerPtr joint2_Controller_;
        physics::JointControllerPtr joint3_Controller_;
        physics::JointControllerPtr joint4_Controller_;
        physics::JointControllerPtr joint5_Controller_;
        physics::JointControllerPtr joint6_Controller_;
        physics::JointControllerPtr joint7_Controller_;
        physics::JointControllerPtr joint8_Controller_;
        physics::JointControllerPtr joint9_Controller_;
        physics::JointControllerPtr joint10_Controller_;
        physics::JointControllerPtr joint11_Controller_;
        physics::JointControllerPtr joint12_Controller_;

        physics::JointPtr joint1_;
        physics::JointPtr joint2_;
        physics::JointPtr joint3_;
        physics::JointPtr joint4_;
        physics::JointPtr joint5_;
        physics::JointPtr joint6_;
        physics::JointPtr joint7_;
        physics::JointPtr joint8_;
        physics::JointPtr joint9_;
        physics::JointPtr joint10_;
        physics::JointPtr joint11_;
        physics::JointPtr joint12_;



        // physics::LinkPtr cell1_;
        // physics::LinkPtr cell2_;
        // physics::LinkPtr cell3_;
        // physics::LinkPtr cell4_;



        event::ConnectionPtr updateConnection;
        rclcpp::Node::SharedPtr node;

        std::deque<bool> LF_contact_history;
        std::deque<bool> RF_contact_history;
        std::deque<bool> LB_contact_history;
        std::deque<bool> RB_contact_history;

        std::deque<double> LF_distance_history;
        std::deque<double> RF_distance_history;
        std::deque<double> LB_distance_history;
        std::deque<double> RB_distance_history;
    };

    GZ_REGISTER_MODEL_PLUGIN(aidin_m1_plugin)
}

#endif
