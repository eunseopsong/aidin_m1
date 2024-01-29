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
        //private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointtorque;

        // Pointer to model
        private: physics::ModelPtr model;

        // Pointer to the joint controller
        private: physics::JointControllerPtr joint1_Controller_;
        private: physics::JointControllerPtr joint2_Controller_;
        private: physics::JointControllerPtr joint3_Controller_;
        private: physics::JointControllerPtr joint4_Controller_;

        // Pointer to the joint
        private: physics::JointPtr joint1_;
        private: physics::JointPtr joint2_;
        private: physics::JointPtr joint3_;
        private: physics::JointPtr joint4_;

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
            this->joint1_ = this->model->GetJoint("aidin_m1::RFJ_scap");
            this->joint2_ = this->model->GetJoint("aidin_m1::LFJ_scap");
            this->joint3_ = this->model->GetJoint("aidin_m1::LBJ_scap");
            this->joint4_ = this->model->GetJoint("aidin_m1::RBJ_scap");

            // Store the joint Controller to control Joint
            this->joint1_Controller_ = this->model->GetJointController();
            this->joint2_Controller_ = this->model->GetJointController();
            this->joint3_Controller_ = this->model->GetJointController();
            this->joint4_Controller_ = this->model->GetJointController();

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
            /*this->pub_jointtorque = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(
                robot_namespace+"JointTorque_sim", qos);
            */

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
            JointPos.data.push_back(this->joint1_->Position(1));
            JointPos.data.push_back(this->joint2_->Position(1));
            JointPos.data.push_back(this->joint3_->Position(1));
            JointPos.data.push_back(this->joint4_->Position(1));
            pub_jointpos->publish(JointPos);


            // Publish joint velocity
            std_msgs::msg::Float32MultiArray JointVel;
            JointVel.data.clear();
            JointVel.data.push_back(this->joint1_->GetVelocity(1));
            JointVel.data.push_back(this->joint2_->GetVelocity(1));
            JointVel.data.push_back(this->joint3_->GetVelocity(1));
            JointVel.data.push_back(this->joint4_->GetVelocity(1));
            pub_jointvel->publish(JointVel);
	/*
            // Publish joint torque
            std_msgs::msg::Float32MultiArray JointTorque;
            JointTorque.data.clear();
            JointTorque.data.push_back(this->joint1_->GetForceTorque(0).body2Torque[1]);
            JointTorque.data.push_back(this->joint2_->GetForceTorque(0).body2Torque[1]);
            JointTorque.data.push_back(this->joint3_->GetForceTorque(0).body2Torque[1]);
            pub_jointtorque->publish(JointTorque);
	*/

            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(this->node);
            executor.spin_once();
        }

        void ROSCallbackTorque_sim(const std_msgs::msg::Float32MultiArray::ConstSharedPtr torque)
        {
            this->joint1_->SetForce(0, torque->data[0]);
            this->joint2_->SetForce(0, torque->data[1]);
            this->joint3_->SetForce(0, torque->data[2]);
            this->joint4_->SetForce(0, torque->data[3]);
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(aidin_m1_plugin);
}
#endif
