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
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bodypose;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointpos;
        private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_jointvel;
        // private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bodypos;
        // private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_bodyvel;
        // private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_imu;
        // private: rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_contact;


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


        public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->model = _model;

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
            this->pub_bodypose = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"BodyPose_sim", qos);
            this->pub_jointpos = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"JointPos_sim", qos);
            this->pub_jointvel = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"JointVel_sim", qos);
            // this->pub_bodypos  = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"BodyPos_sim", qos);
            // this->pub_bodyvel  = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"BodyVel_sim", qos);
            // this->pub_imu      = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"IMU_sim", qos);
            // this->pub_contact  = this->node->create_publisher<std_msgs::msg::Float32MultiArray>(robot_namespace+"Contact_sim", qos);

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&aidin_m1_plugin::OnUpdate, this));
        }

        // Called by the world update start event
        public: void OnUpdate()
        {
            // get robot state from gazebo
            ignition::math::Pose3<double>  WorldPose = model->WorldPose();
            ignition::math::Vector3<double> Pos = WorldPose.Pos(); // body position
            ignition::math::Vector3<double> Rot_Euler = WorldPose.Rot().Euler(); // body euler angle position
            ignition::math::Vector3<double> WorldLinearAccel = model->WorldLinearAccel(); // body linear acceleration from world
            ignition::math::Vector3<double> WorldLinearVel = model->WorldLinearVel(); // body linear velocity from world
            ignition::math::Vector3<double> WorldAngularVel = model->WorldAngularVel(); // body angle velocity from world
            ignition::math::Vector3<double> WorldAngularAccel = model->WorldAngularAccel(); // body euler angle acceleration from world
            ignition::math::Vector3<double> RelativeAngularVel = model->RelativeAngularVel(); // body angular velocity from robot
            // ignition::math::Vector3<double> acc_rel = model->WorldLinearAccel();
            // ignition::math::Pose3<double>  model_pose_rel = model->RelativePose();
            // ignition::math::Vector3<double> rot_rel = model_pose_rel.Rot().Euler();

            // publish body position and euler angle
            std_msgs::msg::Float32MultiArray BodyPose;
            BodyPose.data.clear();
            BodyPose.data.push_back(Pos.X());
            BodyPose.data.push_back(Pos.Y());
            BodyPose.data.push_back(Pos.Z());
            BodyPose.data.push_back(Rot_Euler.X());
            BodyPose.data.push_back(Rot_Euler.Y());
            BodyPose.data.push_back(Rot_Euler.Z());
            pub_bodypose.publish(BodyPose);

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

            // publish body position
            // std_msgs::msg::Float32MultiArray BodyPos;
            // BodyPos.data.clear();
            // BodyPos.data.push_back(Pos.X());
            // BodyPos.data.push_back(Pos.Y());
            // BodyPos.data.push_back(Pos.Z());
            // pub_bodypos.publish(BodyPos);

            // publish body velocity
            // std_msgs::msg::Float32MultiArray BodyVel;
            // BodyVel.data.clear();
	        // BodyVel.data.push_back(WorldLinearVel.X());
	        // BodyVel.data.push_back(WorldLinearVel.Y());
	        // BodyVel.data.push_back(WorldLinearVel.Z());
	        // pub_bodyvel.publish(BodyVel);

            // publish imu
            // std_msgs::msg::Float32MultiArray IMU;
            // IMU.data.clear();
            // IMU.data.push_back(Rot_Euler.X()); // roll, pitch, yaw
            // IMU.data.push_back(Rot_Euler.Y());
            // IMU.data.push_back(Rot_Euler.Z());
            // IMU.data.push_back(RelativeAngularVel.X()); // roll, pitch, yaw velocity
            // IMU.data.push_back(RelativeAngularVel.Y());
            // IMU.data.push_back(RelativeAngularVel.Z());
            // IMU.data.push_back(WorldLinearAccel.X()); // roll, pitch, yaw acceleration
            // IMU.data.push_back(WorldLinearAccel.Y());
            // IMU.data.push_back(WorldLinearAccel.Z());
            // pub_imu.publish(IMU);

            // get and publish contact states
            // std::string contact_link;
            // double LF_contactFlag = 0;
            // double RF_contactFlag = 0;
            // double LB_contactFlag = 0;
            // double RB_contactFlag = 0;

            // physics::ContactManager *contactManager = this->model->GetWorld()->Physics()->GetContactManager();
            // for(int i=0; i<contactManager->GetContactCount(); i++)
            // {
            //     physics::Contact *contact = contactManager->GetContact(i);
            //     contact_link = contact->collision1->GetLink()->GetName();
            //     if(contact_link == "LF_knee"){LF_contactFlag = 1;}
            //     if(contact_link == "RF_knee"){RF_contactFlag = 1;}
            //     if(contact_link == "RB_knee"){LB_contactFlag = 1;}
            //     if(contact_link == "LB_knee"){RB_contactFlag = 1;}
            // }

            // std_msgs::msg::Float32MultiArray Contact;
            // Contact.data.clear();
            // Contact.data.push_back(LF_contactFlag);
            // Contact.data.push_back(RF_contactFlag);
            // Contact.data.push_back(LB_contactFlag);
            // Contact.data.push_back(RB_contactFlag);
            // pub_contact.publish(Contact);

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
