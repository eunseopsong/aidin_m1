#ifndef _LEG_PLUGIN_HH_
#define _LEG_PLUGIN_HH_

#include "rclcpp/rclcpp.hpp"

#include <stdio.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std;

namespace gazebo
{
    class aidin_m1_plugin : public ModelPlugin
    {
        // Constructor
        public: aidin_m1_plugin() {
            int argc =0;
            // rclcpp::init(argc, nullptr);
        }

        // Destructor
        public: ~aidin_m1_plugin() {}

        // Pointer to subscriber
        private: rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_torque;
        // Pointer to publisher
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointpos;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointvel;

        // Pointer to model
        private: physics::ModelPtr model;

        // Pointer to the joint controller
        private: physics::JointControllerPtr joint1_Controller_;
        private: physics::JointControllerPtr joint2_Controller_;
        private: physics::JointControllerPtr joint3_Controller_;
        private: physics::JointControllerPtr joint4_Controller_;
        private: physics::JointControllerPtr joint5_Controller_;
        private: physics::JointControllerPtr joint6_Controller_;
        private: physics::JointControllerPtr joint7_Controller_;
        private: physics::JointControllerPtr joint8_Controller_;
        private: physics::JointControllerPtr joint9_Controller_;
        private: physics::JointControllerPtr joint10_Controller_;
        private: physics::JointControllerPtr joint11_Controller_;
        private: physics::JointControllerPtr joint12_Controller_;

        // Pointer to the joint
        private: physics::JointPtr joint1_;
        private: physics::JointPtr joint2_;
        private: physics::JointPtr joint3_;
        private: physics::JointPtr joint4_;
        private: physics::JointPtr joint5_;
        private: physics::JointPtr joint6_;
        private: physics::JointPtr joint7_;
        private: physics::JointPtr joint8_;
        private: physics::JointPtr joint9_;
        private: physics::JointPtr joint10_;
        private: physics::JointPtr joint11_;
        private: physics::JointPtr joint12_;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        // Start node
        private: rclcpp::Node::SharedPtr node;


        public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model = _model;

            // Initialize node
            node = rclcpp::Node::make_shared("my_aidin_m1_node");

            // Print messages
            RCLCPP_INFO(this->node->get_logger(), "aiidn_m1_plugin run");
            std::cerr << "The aidin_m1 plugin is attached to the model ["
                      << this->model->GetName() << "]\n";

            std::string robot_namespace = "/" + this->model->GetName() + "/";

/*
            // Print joint info
            int jointCount = this->model->GetJointCount();
            RCLCPP_INFO(node->get_logger(), "Number of joints in the model: %d", jointCount);

            for (int i = 0; i < jointCount; ++i) {
                physics::JointPtr joint = this->model->GetJoints()[i];
                RCLCPP_INFO(node->get_logger(), "Joint %d: %s", i, joint->GetName().c_str());
            }
*/

            // Store the joint
            this->joint1_  = this->model->GetJoint("aidin_m1::LFJ_scap");
            this->joint2_  = this->model->GetJoint("aidin_m1::LFJ_hip");
            this->joint3_  = this->model->GetJoint("aidin_m1::LFJ_knee");
            this->joint4_  = this->model->GetJoint("aidin_m1::RFJ_scap");
            this->joint5_  = this->model->GetJoint("aidin_m1::RFJ_hip");
            this->joint6_  = this->model->GetJoint("aidin_m1::RFJ_knee");
            this->joint7_  = this->model->GetJoint("aidin_m1::LBJ_scap");
            this->joint8_  = this->model->GetJoint("aidin_m1::LBJ_hip");
            this->joint9_  = this->model->GetJoint("aidin_m1::LBJ_knee");
            this->joint10_ = this->model->GetJoint("aidin_m1::RBJ_scap");
            this->joint11_ = this->model->GetJoint("aidin_m1::RBJ_hip");
            this->joint12_ = this->model->GetJoint("aidin_m1::RBJ_knee");

            // Store the joint Controller to control Joint
            this->joint1_Controller_  = this->model->GetJointController();
            this->joint2_Controller_  = this->model->GetJointController();
            this->joint3_Controller_  = this->model->GetJointController();
            this->joint4_Controller_  = this->model->GetJointController();
            this->joint5_Controller_  = this->model->GetJointController();
            this->joint6_Controller_  = this->model->GetJointController();
            this->joint7_Controller_  = this->model->GetJointController();
            this->joint8_Controller_  = this->model->GetJointController();
            this->joint9_Controller_  = this->model->GetJointController();
            this->joint10_Controller_ = this->model->GetJointController();
            this->joint11_Controller_ = this->model->GetJointController();
            this->joint12_Controller_ = this->model->GetJointController();

            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
            //qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            //qos.transient_local();

            // Create subscriber
            this->sub_torque = this->node->create_subscription<std_msgs::msg::Float32MultiArray>(
                robot_namespace+"Torque_sim",
                qos,
                std::bind(&aidin_m1_plugin::ROSCallbackTorque_sim, this, std::placeholders::_1));

            // Create publisher
            this->pub_jointpos = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(
                robot_namespace+"JointPos_sim", qos);
            this->pub_jointvel = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(
                robot_namespace+"JointVel_sim", qos);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&aidin_m1_plugin::OnUpdate, this));
        }

        public: void OnUpdate()
        {
            // Publish joint position
            std_msgs::msg::Float32MultiArray JointPos;
            JointPos.data.clear();
            JointPos.data.push_back(this->joint1_ ->Position(1));
            JointPos.data.push_back(this->joint2_ ->Position(1));
            JointPos.data.push_back(this->joint3_ ->Position(1));
            JointPos.data.push_back(this->joint4_ ->Position(1));
            JointPos.data.push_back(this->joint5_ ->Position(1));
            JointPos.data.push_back(this->joint6_ ->Position(1));
            JointPos.data.push_back(this->joint7_ ->Position(1));
            JointPos.data.push_back(this->joint8_ ->Position(1));
            JointPos.data.push_back(this->joint9_ ->Position(1));
            JointPos.data.push_back(this->joint10_->Position(1));
            JointPos.data.push_back(this->joint11_->Position(1));
            JointPos.data.push_back(this->joint12_->Position(1));
            pub_jointpos->publish(JointPos);


            // Publish joint velocity
            std_msgs::msg::Float32MultiArray JointVel;
            JointVel.data.clear();
            JointVel.data.push_back(this->joint1_ ->GetVelocity(1));
            JointVel.data.push_back(this->joint2_ ->GetVelocity(1));
            JointVel.data.push_back(this->joint3_ ->GetVelocity(1));
            JointVel.data.push_back(this->joint4_ ->GetVelocity(1));
            JointVel.data.push_back(this->joint5_ ->GetVelocity(1));
            JointVel.data.push_back(this->joint6_ ->GetVelocity(1));
            JointVel.data.push_back(this->joint7_ ->GetVelocity(1));
            JointVel.data.push_back(this->joint8_ ->GetVelocity(1));
            JointVel.data.push_back(this->joint9_ ->GetVelocity(1));
            JointVel.data.push_back(this->joint10_->GetVelocity(1));
            JointVel.data.push_back(this->joint11_->GetVelocity(1));
            JointVel.data.push_back(this->joint12_->GetVelocity(1));
            pub_jointvel->publish(JointVel);

            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(this->node);
            executor.spin_once();
        }

        void ROSCallbackTorque_sim(const std_msgs::msg::Float32MultiArray::ConstSharedPtr torque)
        {
            this->joint1_ ->SetForce(0, torque->data[0]);
            this->joint2_ ->SetForce(0, torque->data[1]);
            this->joint3_ ->SetForce(0, torque->data[2]);
            this->joint4_ ->SetForce(0, torque->data[3]);
            this->joint5_ ->SetForce(0, torque->data[4]);
            this->joint6_ ->SetForce(0, torque->data[5]);
            this->joint7_ ->SetForce(0, torque->data[6]);
            this->joint8_ ->SetForce(0, torque->data[7]);
            this->joint9_ ->SetForce(0, torque->data[8]);
            this->joint10_->SetForce(0, torque->data[9]);
            this->joint11_->SetForce(0, torque->data[10]);
            this->joint12_->SetForce(0, torque->data[11]);
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(aidin_m1_plugin);
}
#endif
