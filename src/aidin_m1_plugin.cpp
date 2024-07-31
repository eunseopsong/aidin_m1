#ifndef _LEG_PLUGIN_HH_
#define _LEG_PLUGIN_HH_

#include "rclcpp/rclcpp.hpp"

#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <deque>
#include <numeric>

// Define history size and threshold
const size_t contact_history_size = 7;
const double contact_threshold = 0.15; // Lowered threshold to make it more likely to be 1

// Define weights for filtering
// const std::vector<double> contact_weights = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
const std::vector<double> contact_weights = {0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.0};

using namespace std;

namespace gazebo
{
    class aidin_m1_plugin : public ModelPlugin
    {
        // Constructor
        public: aidin_m1_plugin() {
            // int argc =0;
            // rclcpp::init(argc, nullptr);
        }

        // Destructor
        public: ~aidin_m1_plugin() {}

        // Pointer to subscriber
        private: rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_torque;
        // Pointer to publisher
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bodypose;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointpos;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointvel;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bodypos;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bodyvel;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_imu;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_contact;


        // Pointer to the model
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
        // private: physics::JointControllerPtr joint13_Controller_;

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
        // private: physics::JointPtr joint13_;
        // private: physics::LinkPtr link1_;
        // private: physics::LinkPtr plane1_;
        // neccessary?

        // Pointer to the body
        private: physics::LinkPtr globalposition;
        // private: physics::LinkPtr imusensor;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
        // private: std::vector<event::ConnectionPtr> updateConnection; // Compare; aidin8 (ROS1)

        // Start node
        private: rclcpp::Node::SharedPtr node;


        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Initialize node
            node = rclcpp::Node::make_shared("my_aidin_m1_node");

            // Print messages
            RCLCPP_INFO(this->node->get_logger(), "aiidn_m1_plugin run");
            std::cerr << "The aidin_m1 plugin is attached to the model ["
                      << this->model->GetName() << "]\n";

            std::string robot_namespace = "/" + this->model->GetName() + "/";

            // // Print joint info
            // int jointCount = this->model->GetJointCount();
            // RCLCPP_INFO(node->get_logger(), "Number of joints in the model: %d", jointCount);

            // for (int i = 0; i < jointCount; ++i) {
            //     physics::JointPtr joint = this->model->GetJoints()[i];
            //     RCLCPP_INFO(node->get_logger(), "Joint %d: %s", i, joint->GetName().c_str());
            // }

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
            // this->joint13_ = this->model->GetJoint("aidin_m1::z_joint_revolute");

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
            // this->joint13_Controller_ = this->model->GetJointController();

            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
            //qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
            //qos.transient_local();

            // Create subscriber
            this->sub_torque = this->node->create_subscription<std_msgs::msg::Float32MultiArray>(
                robot_namespace+"Torque_sim",
                qos,
                std::bind(&aidin_m1_plugin::ROSCallbackTorque_sim, this, std::placeholders::_1));

            // Create publisher
            this->pub_bodypose = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"BodyPose_sim", qos);
            this->pub_jointpos = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"JointPos_sim", qos);
            this->pub_jointvel = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"JointVel_sim", qos);
            this->pub_bodypos  = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"BodyPos_sim", qos);
            this->pub_bodyvel  = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"BodyVel_sim", qos);
            this->pub_imu      = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"IMU_sim", qos);
            this->pub_contact  = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"Contact_sim", qos);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&aidin_m1_plugin::OnUpdate, this));
        }

        private:
            std::deque<bool> LF_contact_history;
            std::deque<bool> RF_contact_history;
            std::deque<bool> LB_contact_history;
            std::deque<bool> RB_contact_history;


        // Called by the world update start event
        public: void OnUpdate() {
            ignition::math::Pose3<double> WorldPose = model->WorldPose();
            ignition::math::Vector3<double> Pos = WorldPose.Pos();
            ignition::math::Vector3<double> Rot_Euler = WorldPose.Rot().Euler();
            ignition::math::Vector3<double> WorldLinearVel = model->WorldLinearVel();
            ignition::math::Vector3<double> WorldLinearAccel = model->WorldLinearAccel();
            ignition::math::Vector3<double> RelativeAngularVel = model->RelativeAngularVel();

            std_msgs::msg::Float32MultiArray BodyPose;
            BodyPose.data.clear();
            BodyPose.data.push_back(Pos.X());
            BodyPose.data.push_back(Pos.Y());
            BodyPose.data.push_back(Pos.Z());
            BodyPose.data.push_back(Rot_Euler.X());
            BodyPose.data.push_back(Rot_Euler.Y());
            BodyPose.data.push_back(Rot_Euler.Z());
            pub_bodypose->publish(BodyPose);

            std_msgs::msg::Float32MultiArray JointPos;
            JointPos.data.clear();
            JointPos.data.push_back(this->joint1_->Position(1));
            JointPos.data.push_back(this->joint2_->Position(1));
            JointPos.data.push_back(this->joint3_->Position(1));
            JointPos.data.push_back(this->joint4_->Position(1));
            JointPos.data.push_back(this->joint5_->Position(1));
            JointPos.data.push_back(this->joint6_->Position(1));
            JointPos.data.push_back(this->joint7_->Position(1));
            JointPos.data.push_back(this->joint8_->Position(1));
            JointPos.data.push_back(this->joint9_->Position(1));
            JointPos.data.push_back(this->joint10_->Position(1));
            JointPos.data.push_back(this->joint11_->Position(1));
            JointPos.data.push_back(this->joint12_->Position(1));
            pub_jointpos->publish(JointPos);

            std_msgs::msg::Float32MultiArray JointVel;
            JointVel.data.clear();
            JointVel.data.push_back(this->joint1_->GetVelocity(1));
            JointVel.data.push_back(this->joint2_->GetVelocity(1));
            JointVel.data.push_back(this->joint3_->GetVelocity(1));
            JointVel.data.push_back(this->joint4_->GetVelocity(1));
            JointVel.data.push_back(this->joint5_->GetVelocity(1));
            JointVel.data.push_back(this->joint6_->GetVelocity(1));
            JointVel.data.push_back(this->joint7_->GetVelocity(1));
            JointVel.data.push_back(this->joint8_->GetVelocity(1));
            JointVel.data.push_back(this->joint9_->GetVelocity(1));
            JointVel.data.push_back(this->joint10_->GetVelocity(1));
            JointVel.data.push_back(this->joint11_->GetVelocity(1));
            JointVel.data.push_back(this->joint12_->GetVelocity(1));
            pub_jointvel->publish(JointVel);

            std_msgs::msg::Float32MultiArray BodyPos;
            BodyPos.data.clear();
            BodyPos.data.push_back(Pos.X());
            BodyPos.data.push_back(Pos.Y());
            BodyPos.data.push_back(Pos.Z());
            pub_bodypos->publish(BodyPos);

            std_msgs::msg::Float32MultiArray BodyVel;
            BodyVel.data.clear();
            BodyVel.data.push_back(WorldLinearVel.X());
            BodyVel.data.push_back(WorldLinearVel.Y());
            BodyVel.data.push_back(WorldLinearVel.Z());
            pub_bodyvel->publish(BodyVel);

            std_msgs::msg::Float32MultiArray IMU;
            IMU.data.clear();
            IMU.data.push_back(Rot_Euler.X());
            IMU.data.push_back(Rot_Euler.Y());
            IMU.data.push_back(Rot_Euler.Z());
            IMU.data.push_back(RelativeAngularVel.X());
            IMU.data.push_back(RelativeAngularVel.Y());
            IMU.data.push_back(RelativeAngularVel.Z());
            IMU.data.push_back(WorldLinearAccel.X());
            IMU.data.push_back(WorldLinearAccel.Y());
            IMU.data.push_back(WorldLinearAccel.Z());
            pub_imu->publish(IMU);

            physics::ContactManager *contactManager = this->model->GetWorld()->Physics()->GetContactManager();
            RCLCPP_INFO(this->node->get_logger(), "Contact count: %d", contactManager->GetContactCount());

            std::string contact_link;
            bool LF_contact_detected = false;
            bool RF_contact_detected = false;
            bool LB_contact_detected = false;
            bool RB_contact_detected = false;

            for (unsigned int i = 0; i < contactManager->GetContactCount(); i++) {
                physics::Contact *contact = contactManager->GetContact(i);
                if (contact == nullptr) {
                    RCLCPP_WARN(this->node->get_logger(), "Null contact object at index %d", i);
                    continue;
                }

                if (contact->collision1 && contact->collision2) {
                    std::string link1 = contact->collision1->GetLink()->GetName();
                    std::string link2 = contact->collision2->GetLink()->GetName();
                    RCLCPP_INFO(this->node->get_logger(), "Collision between %s and %s", link1.c_str(), link2.c_str());

                    if (link1 == "LF_knee" || link2 == "LF_knee") {
                        LF_contact_detected = true;
                        // RCLCPP_INFO(this->node->get_logger(), "LF_contact_detected");
                    }
                    if (link1 == "RF_knee" || link2 == "RF_knee") {
                        RF_contact_detected = true;
                        // RCLCPP_INFO(this->node->get_logger(), "RF_contact_detected");
                    }
                    if (link1 == "LB_knee" || link2 == "LB_knee") {
                        LB_contact_detected = true;
                        // RCLCPP_INFO(this->node->get_logger(), "LB_contact_detected");
                    }
                    if (link1 == "RB_knee" || link2 == "RB_knee") {
                        RB_contact_detected = true;
                        // RCLCPP_INFO(this->node->get_logger(), "RB_contact_detected");
                    }
                } else {
                    RCLCPP_WARN(this->node->get_logger(), "Collision object does not have valid links");
                }
            }

            // Update contact history
            if (LF_contact_history.size() >= contact_history_size) LF_contact_history.pop_front();
            if (RF_contact_history.size() >= contact_history_size) RF_contact_history.pop_front();
            if (LB_contact_history.size() >= contact_history_size) LB_contact_history.pop_front();
            if (RB_contact_history.size() >= contact_history_size) RB_contact_history.pop_front();

            LF_contact_history.push_back(LF_contact_detected);
            RF_contact_history.push_back(RF_contact_detected);
            LB_contact_history.push_back(LB_contact_detected);
            RB_contact_history.push_back(RB_contact_detected);

            // Calculate weighted contact detection
            auto weighted_contact = [](const std::deque<bool>& history, const std::vector<double>& weights) {
                double sum = 0.0;
                for (size_t i = 0; i < history.size(); ++i) {
                    sum += history[i] * weights[i];
                }
                return sum;
            };

            double LF_contact_weighted = weighted_contact(LF_contact_history, contact_weights);
            double RF_contact_weighted = weighted_contact(RF_contact_history, contact_weights);
            double LB_contact_weighted = weighted_contact(LB_contact_history, contact_weights);
            double RB_contact_weighted = weighted_contact(RB_contact_history, contact_weights);

            double LF_contactFlag = LF_contact_weighted > contact_threshold ? 1 : 0;
            double RF_contactFlag = RF_contact_weighted > contact_threshold ? 1 : 0;
            double LB_contactFlag = LB_contact_weighted > contact_threshold ? 1 : 0;
            double RB_contactFlag = RB_contact_weighted > contact_threshold ? 1 : 0;

            RCLCPP_INFO(this->node->get_logger(), "LF_contactFlag: %f, RF_contactFlag: %f, LB_contactFlag: %f, RB_contactFlag: %f", LF_contactFlag, RF_contactFlag, LB_contactFlag, RB_contactFlag);

            std_msgs::msg::Float32MultiArray Contact;
            Contact.data.clear();
            Contact.data.push_back(LF_contactFlag);
            Contact.data.push_back(RF_contactFlag);
            Contact.data.push_back(LB_contactFlag);
            Contact.data.push_back(RB_contactFlag);
            pub_contact->publish(Contact);

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
            // this->joint13_->SetForce(0, torque->data[12]);
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(aidin_m1_plugin)
}
#endif
