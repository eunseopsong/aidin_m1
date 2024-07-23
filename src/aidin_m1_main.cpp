// #include "FootstepPlanner.cpp"
#include "func.cpp"

using namespace std;
using namespace Eigen;
using namespace std::chrono_literals;

class JointControl : public rclcpp::Node
{
public:
    JointControl()
        : Node("aidin_m1_control_node"), count_(0)
    {
        // Subscribe to topics
        sub_bodypose = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/BodyPose_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                copy(msg->data.begin(), msg->data.begin() + 6, body_pose.begin());
            });
        sub_jointpos = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/JointPos_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                copy(msg->data.begin(), msg->data.begin() + 12, joint_pos.begin());
            });
        sub_jointvel = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/JointVel_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                copy(msg->data.begin(), msg->data.begin() + 12, joint_vel.begin());
            });
        sub_bodypos = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/BodyPos_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                copy(msg->data.begin(), msg->data.begin() + 3, body_pos.begin());
            });
        sub_bodyvel = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/BodyVel_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                copy(msg->data.begin(), msg->data.begin() + 3, body_vel.begin());
            });
        sub_imu = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/IMU_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                copy(msg->data.begin(), msg->data.begin() + 9, imu.begin());
            });
        sub_contact = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Contact_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                copy(msg->data.begin(), msg->data.begin() + 4, contact.begin());
            });
        sub_command = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Command_sim", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                copy(msg->data.begin(), msg->data.begin() + 3, command.begin());
            });

        // Publish torque & target joint poses
        pub_torque = this->create_publisher<std_msgs::msg::Float32MultiArray>("/aidin_m1/Torque_sim", 10);
        pub_targetpos = this->create_publisher<std_msgs::msg::Float32MultiArray>("/aidin_m1/TargetPos", 10);

        // node Publish 실행 주기 설정 (1ms)
        timer_ = this->create_wall_timer(1ms, std::bind(&JointControl::CalculateAndPublishTorque, this));
    }

private:
    void CalculateAndPublishTorque()
    {
        count_ += 0.001; // Increment simulation time by 1ms

        // Gain Initialization
        Matrix3d Kp, Kd;
        //    Initial_Standing  SWing_Phase  STanding_Phase
        Kp <<       600,            600,            1200,   // Scapula
                   4000,           4000,            8000,   // Hip
                  26000,          26000,           52000;   // Knee
        Kd <<        20,             20,              40,   // Scapula
                     20,             20,              40,   // Hip
                     10,             10,              20;   // Knee

        double vel_of_body = command[1]; // Target velocity of the whole robot body (mm/s)
        double T = command[2];           // Period of the whole trajectory phase    (sec)
        double t = fmod(count_, T);
        double t_counter = fmod(count_ + T/2, T);

        // Calculate the coordinate using Trajectory Function
        double yVal = 95;
        double xVal, zVal;
        SplineTrajectory(t, T, vel_of_body, xVal, zVal);

        double xVal_counter, zVal_counter;
        SplineTrajectory(t_counter, T, vel_of_body, xVal_counter, zVal_counter);

        // Calculate the target_pos using Inverse Kinematics
        array<double, 3> LF_target_pos, RF_target_pos, LB_target_pos, RB_target_pos;

        InverseKinematics3D(yVal, zVal, xVal, yVal, 250, 250, LF_target_pos.data());
        InverseKinematics3D(yVal, zVal_counter, xVal_counter, yVal, 250, 250, RF_target_pos.data());
        InverseKinematics3D(yVal, zVal_counter, xVal_counter, yVal, 250, 250, LB_target_pos.data());
        InverseKinematics3D(yVal, zVal, xVal, yVal, 250, 250, RB_target_pos.data());

        vector<double> target_pos;
        target_pos.insert(target_pos.end(), LF_target_pos.begin(), LF_target_pos.end());
        target_pos.insert(target_pos.end(), RF_target_pos.begin(), RF_target_pos.end());
        target_pos.insert(target_pos.end(), LB_target_pos.begin(), LB_target_pos.end());
        target_pos.insert(target_pos.end(), RB_target_pos.begin(), RB_target_pos.end());

        // Output torque to pubilsh Initialization
        vector<double> output_torque(12);

        switch (int(command[0])) {
            case 1: // the command to keep a robot "standing still"
                CalculateTorqueStanding(output_torque.data(), {Kp(0,0), Kp(1,0), Kp(2,0)}, {Kd(0,0), Kd(1,0), Kd(2,0)});
                break;
            case 2: // the command to keep a robot "walking in place"
                // CalculateTorqueWalkingInPlace(ouput_torque);
                break;
            case 3: // the command to make a robot "run"
                CalculateTorqueRunning(output_torque);
                break;
            default:
                fill(output_torque.begin(), output_torque.end(), 0.0);
                break;
        }

        // for (int i=0; i<12; i++)
        // {
        //     else if (command[0] == 3) // running command
        //     {
        //         if (i<3)      // LF joint
        //         {
        //             output_torque[i] = FeedforwardController(Kp(i,0), Kd(i,0), LF_target_pos.data(), i, 0);
        //         }
        //         else if (i<6) // RF joint
        //         {
        //             output_torque[i] = FeedforwardController(Kp(i-3,0), Kd(i-3,0), RF_target_pos.data(), i-3, 3);
        //         }
        //         else if (i<9) // LB joint
        //         {
        //             output_torque[i] = FeedforwardController(Kp(i-6,0), Kd(i-6,0), LB_target_pos.data(), i-6, 6);
        //         }
        //         else          // RB joint
        //         {
        //             output_torque[i] = FeedforwardController(Kp(i-9,0), Kd(i-9,0), RB_target_pos.data(), i-9, 9);
        //         }
        //     }
        //     else
        //         output_torque[i] = 0;
        // }

        // Publish Desired Pose
        std_msgs::msg::Float32MultiArray targetpos_msg;
        targetpos_msg.data.insert(targetpos_msg.data.end(), target_pos.begin(), target_pos.end());
        pub_targetpos->publish(targetpos_msg);

        // Publish Torque
        std_msgs::msg::Float32MultiArray torque_msg;
        torque_msg.data.insert(torque_msg.data.end(), output_torque.begin(), output_torque.end());
        pub_torque->publish(torque_msg);
    }

    // Subscriber declarations
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_bodypose;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointpos;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointvel;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_bodypos;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_bodyvel;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_imu;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_contact;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_command;

    // Publisher declarations
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_torque;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_targetpos;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    double count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
