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
const size_t contact_history_size = 10;
const double contact_threshold = 0.1;

// Define history size for distance filtering
const size_t distance_history_size = 5;

// Define weights for filtering
const std::vector<double> contact_weights = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

using namespace std;

namespace gazebo
{
    class aidin_m1_plugin : public ModelPlugin
    {
        public: aidin_m1_plugin() {}

        public: ~aidin_m1_plugin() {}

        private: rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_torque;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bodypose;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointpos;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointvel;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bodypos;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bodyvel;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_imu;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_contact;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_distance; // for distance

        private: physics::ModelPtr model;

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

        private: event::ConnectionPtr updateConnection;

        private: rclcpp::Node::SharedPtr node;

        private: std::deque<bool> LF_contact_history;
        private: std::deque<bool> RF_contact_history;
        private: std::deque<bool> LB_contact_history;
        private: std::deque<bool> RB_contact_history;

        private: std::deque<double> LF_distance_history; // for distance
        private: std::deque<double> RF_distance_history; // for distance
        private: std::deque<double> LB_distance_history; // for distance
        private: std::deque<double> RB_distance_history; // for distance

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;
            node = rclcpp::Node::make_shared("my_aidin_m1_node");
            RCLCPP_INFO(this->node->get_logger(), "aiidn_m1_plugin run");
            std::cerr << "The aidin_m1 plugin is attached to the model [" << this->model->GetName() << "]\n";

            std::string robot_namespace = "/" + this->model->GetName() + "/";

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

            this->sub_torque = this->node->create_subscription<std_msgs::msg::Float32MultiArray>(
                robot_namespace+"Torque_sim",
                qos,
                std::bind(&aidin_m1_plugin::ROSCallbackTorque_sim, this, std::placeholders::_1));

            this->pub_bodypose = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"BodyPose_sim", qos);
            this->pub_jointpos = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"JointPos_sim", qos);
            this->pub_jointvel = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"JointVel_sim", qos);
            this->pub_bodypos  = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"BodyPos_sim", qos);
            this->pub_bodyvel  = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"BodyVel_sim", qos);
            this->pub_imu      = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"IMU_sim", qos);
            this->pub_contact  = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"Contact_sim", qos);
            this->pub_distance = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"Distance_sim", qos); // for distance

            this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&aidin_m1_plugin::OnUpdate, this));
        }

        // Calculate the center of mass
        ignition::math::Vector3d ComputeCenterOfMass() {
            ignition::math::Vector3d com(0, 0, 0);
            double totalMass = 0.0;

            for (auto link : this->model->GetLinks()) {
                double mass = link->GetInertial()->Mass();
                ignition::math::Vector3d pos = link->WorldCoGPose().Pos();
                com += pos * mass;
                totalMass += mass;
            }

            if (totalMass > 0) {
                com /= totalMass;
            }

            return com;
        }

        public: void OnUpdate()
        {
            auto WorldPose = model->WorldPose();
            auto Pos = WorldPose.Pos();
            auto Rot_Euler = WorldPose.Rot().Euler();
            auto WorldLinearVel = model->WorldLinearVel();
            auto WorldLinearAccel = model->WorldLinearAccel();
            auto RelativeAngularVel = model->RelativeAngularVel();

            std_msgs::msg::Float32MultiArray BodyPose;
            BodyPose.data = {static_cast<float>(Pos.X()), static_cast<float>(Pos.Y()), static_cast<float>(Pos.Z()), 
                             static_cast<float>(Rot_Euler.X()), static_cast<float>(Rot_Euler.Y()), static_cast<float>(Rot_Euler.Z())};
            pub_bodypose->publish(BodyPose);

            std_msgs::msg::Float32MultiArray JointPos;
            JointPos.data = {
                static_cast<float>(this->joint1_->Position(1)), static_cast<float>(this->joint2_->Position(1)), static_cast<float>(this->joint3_->Position(1)),
                static_cast<float>(this->joint4_->Position(1)), static_cast<float>(this->joint5_->Position(1)), static_cast<float>(this->joint6_->Position(1)),
                static_cast<float>(this->joint7_->Position(1)), static_cast<float>(this->joint8_->Position(1)), static_cast<float>(this->joint9_->Position(1)),
                static_cast<float>(this->joint10_->Position(1)), static_cast<float>(this->joint11_->Position(1)), static_cast<float>(this->joint12_->Position(1))
            };
            pub_jointpos->publish(JointPos);

            std_msgs::msg::Float32MultiArray JointVel;
            JointVel.data = {
                static_cast<float>(this->joint1_->GetVelocity(1)), static_cast<float>(this->joint2_->GetVelocity(1)), static_cast<float>(this->joint3_->GetVelocity(1)),
                static_cast<float>(this->joint4_->GetVelocity(1)), static_cast<float>(this->joint5_->GetVelocity(1)), static_cast<float>(this->joint6_->GetVelocity(1)),
                static_cast<float>(this->joint7_->GetVelocity(1)), static_cast<float>(this->joint8_->GetVelocity(1)), static_cast<float>(this->joint9_->GetVelocity(1)),
                static_cast<float>(this->joint10_->GetVelocity(1)), static_cast<float>(this->joint11_->GetVelocity(1)), static_cast<float>(this->joint12_->GetVelocity(1))
            };
            pub_jointvel->publish(JointVel);

            std_msgs::msg::Float32MultiArray BodyPos;
            BodyPos.data = {static_cast<float>(Pos.X()), static_cast<float>(Pos.Y()), static_cast<float>(Pos.Z())};
            pub_bodypos->publish(BodyPos);

            std_msgs::msg::Float32MultiArray BodyVel;
            BodyVel.data = {static_cast<float>(WorldLinearVel.X()), static_cast<float>(WorldLinearVel.Y()), static_cast<float>(WorldLinearVel.Z())};
            pub_bodyvel->publish(BodyVel);

            std_msgs::msg::Float32MultiArray IMU;
            IMU.data = {
                static_cast<float>(Rot_Euler.X()), static_cast<float>(Rot_Euler.Y()), static_cast<float>(Rot_Euler.Z()),
                static_cast<float>(RelativeAngularVel.X()), static_cast<float>(RelativeAngularVel.Y()), static_cast<float>(RelativeAngularVel.Z()),
                static_cast<float>(WorldLinearAccel.X()), static_cast<float>(WorldLinearAccel.Y()), static_cast<float>(WorldLinearAccel.Z())
            };
            pub_imu->publish(IMU);

            auto contactManager = this->model->GetWorld()->Physics()->GetContactManager();
            bool LF_contact_detected = false;
            bool RF_contact_detected = false;
            bool LB_contact_detected = false;
            bool RB_contact_detected = false;

            ignition::math::Vector3d comPos = ComputeCenterOfMass(); // for distance
            double LF_distance = 0.0;
            double RF_distance = 0.0;
            double LB_distance = 0.0;
            double RB_distance = 0.0;

            for (unsigned int i = 0; i < contactManager->GetContactCount(); i++) {
                auto contact = contactManager->GetContact(i);
                if (contact && contact->collision1 && contact->collision2) {
                    auto link1 = contact->collision1->GetLink()->GetName();
                    auto link2 = contact->collision2->GetLink()->GetName();

                    LF_contact_detected |= (link1 == "LF_knee" || link2 == "LF_knee");
                    RF_contact_detected |= (link1 == "RF_knee" || link2 == "RF_knee");
                    LB_contact_detected |= (link1 == "LB_knee" || link2 == "LB_knee");
                    RB_contact_detected |= (link1 == "RB_knee" || link2 == "RB_knee");

                    if (link1 == "LF_knee" || link2 == "LF_knee") {
                        ignition::math::Vector3d contactPos = (link1 == "LF_knee") ? contact->collision1->WorldPose().Pos() : contact->collision2->WorldPose().Pos();
                        LF_distance = comPos.Distance(contactPos);
                    }
                    if (link1 == "RF_knee" || link2 == "RF_knee") {
                        ignition::math::Vector3d contactPos = (link1 == "RF_knee") ? contact->collision1->WorldPose().Pos() : contact->collision2->WorldPose().Pos();
                        RF_distance = comPos.Distance(contactPos);
                    }
                    if (link1 == "LB_knee" || link2 == "LB_knee") {
                        ignition::math::Vector3d contactPos = (link1 == "LB_knee") ? contact->collision1->WorldPose().Pos() : contact->collision2->WorldPose().Pos();
                        LB_distance = comPos.Distance(contactPos);
                    }
                    if (link1 == "RB_knee" || link2 == "RB_knee") {
                        ignition::math::Vector3d contactPos = (link1 == "RB_knee") ? contact->collision1->WorldPose().Pos() : contact->collision2->WorldPose().Pos();
                        RB_distance = comPos.Distance(contactPos);
                    }
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

            auto updateDistanceHistory = [](std::deque<double>& history, double distance) {
                if (history.size() >= distance_history_size) history.pop_front();
                history.push_back(distance);
            };

            updateDistanceHistory(LF_distance_history, LF_distance);
            updateDistanceHistory(RF_distance_history, RF_distance);
            updateDistanceHistory(LB_distance_history, LB_distance);
            updateDistanceHistory(RB_distance_history, RB_distance);

            auto average_distance = [](const std::deque<double>& history) {
                if (history.empty()) return 0.0;
                return std::accumulate(history.begin(), history.end(), 0.0) / history.size();
            };

            double LF_distance_filtered = average_distance(LF_distance_history);
            double RF_distance_filtered = average_distance(RF_distance_history);
            double LB_distance_filtered = average_distance(LB_distance_history);
            double RB_distance_filtered = average_distance(RB_distance_history);

            std_msgs::msg::Float32MultiArray Contact;
            Contact.data = {static_cast<float>(LF_contactFlag), static_cast<float>(RF_contactFlag), static_cast<float>(LB_contactFlag), static_cast<float>(RB_contactFlag)};
            pub_contact->publish(Contact);

            std_msgs::msg::Float32MultiArray Distances;
            Distances.data = {static_cast<float>(LF_distance_filtered), static_cast<float>(RF_distance_filtered), static_cast<float>(LB_distance_filtered), static_cast<float>(RB_distance_filtered)};
            pub_distance->publish(Distances); // for distance

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
            this->joint5_->SetForce(0, torque->data[4]);
            this->joint6_->SetForce(0, torque->data[5]);
            this->joint7_->SetForce(0, torque->data[6]);
            this->joint8_->SetForce(0, torque->data[7]);
            this->joint9_->SetForce(0, torque->data[8]);
            this->joint10_->SetForce(0, torque->data[9]);
            this->joint11_->SetForce(0, torque->data[10]);
            this->joint12_->SetForce(0, torque->data[11]);
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(aidin_m1_plugin)
}
#endif
