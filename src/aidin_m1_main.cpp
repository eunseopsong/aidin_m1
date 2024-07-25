// #include "FootstepPlanner.cpp"
#include "func.cpp"

using namespace std;
using namespace Eigen;
using namespace std::chrono_literals;

class JointControl : public rclcpp::Node
{
public:
    JointControl()
        : Node("aidin_m1_control_node"), _count(0), previous_command{0, 0, 0}
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
                // Check if the command has changed
                if (!equal(command.begin(), command.end(), msg->data.begin())) {
                    _count = 0; // Reset count if command has changed
                    copy(msg->data.begin(), msg->data.begin() + 3, previous_command.begin());
                }
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
        // Check if the command array contains valid values
        if (std::any_of(command.begin(), command.end(), [](double c) { return c != 0; })) {
            _count += 0.001; // Increment simulation time by 1ms
        }

        // Gain Initialization
        Matrix3d Kp, Kd;
        ////  Initial_Standing  SWing_Phase  STanding_Phase
        Kp <<       600,            600,             600, //// Scapula
                   4000,           4000,            6000, //// Hip
                  26000,          26000,           55000; //// Knee
        Kd <<        20,             20,              20, //// Scapula
                     20,             20,              20, //// Hip
                     10,             10,              15; //// Knee

        double vel_of_body = command[1]; // Target velocity of the whole robot body (mm/s)
        double T = command[2];           // Period of the whole trajectory phase    (sec)
        double t = fmod(_count, T);
        double t_counter = fmod(_count + T/2, T);

        RCLCPP_INFO(this->get_logger(), "t: %f, count_: %f", t, _count); // t 값을 디버깅하기 위해 출력

        // Calculate the coordinate using Trajectory Function
        double yVal = 95;
        double xVal, zVal;
        SplineTrajectory(t, T, vel_of_body, xVal, zVal);

        double xVal_counter, zVal_counter;
        SplineTrajectory(t_counter, T, vel_of_body, xVal_counter, zVal_counter);

        // Calculate the target_pos using Inverse Kinematics
        array<double, 3> LF_target_pos, RF_target_pos, LB_target_pos, RB_target_pos;

        switch (int(command[0])) {
            case 1: // the command to keep a robot "standing still"

                break;
            case 2: // the command to keep a robot "walking in place"
                InverseKinematics3D(yVal, zVal, xVal, yVal, 250, 250, LF_target_pos.data());
                InverseKinematics3D(yVal, zVal_counter, xVal_counter, yVal, 250, 250, RF_target_pos.data());
                InverseKinematics3D(yVal, zVal_counter, xVal_counter, yVal, 250, 250, LB_target_pos.data());
                InverseKinematics3D(yVal, zVal, xVal, yVal, 250, 250, RB_target_pos.data());
                break;
            case 3: // the command to make a robot "run" (trotting)
                InverseKinematics3D(yVal, zVal, xVal, yVal, 250, 250, LF_target_pos.data());
                InverseKinematics3D(yVal, zVal_counter, xVal_counter, yVal, 250, 250, RF_target_pos.data());
                InverseKinematics3D(yVal, zVal_counter, xVal_counter, yVal, 250, 250, LB_target_pos.data());
                InverseKinematics3D(yVal, zVal, xVal, yVal, 250, 250, RB_target_pos.data());
                break;
            default:
                break;
        }

        array<double, 12> target_pos;
        copy(LF_target_pos.begin(), LF_target_pos.end(), target_pos.begin());
        copy(RF_target_pos.begin(), RF_target_pos.end(), target_pos.begin() + 3);
        copy(LB_target_pos.begin(), LB_target_pos.end(), target_pos.begin() + 6);
        copy(RB_target_pos.begin(), RB_target_pos.end(), target_pos.begin() + 9);

        // Output torque to pubilsh Initialization
        vector<double> output_torque(12);

        switch (int(command[0])) {
            case 1: // the command to keep a robot "standing still"
                CalculateTorqueStanding(output_torque.data(), {Kp(0,0), Kp(1,0), Kp(2,0)}, {Kd(0,0), Kd(1,0), Kd(2,0)});
                break;
            case 2: // the command to keep a robot "walking in place"
                CalculateTorqueRunning(output_torque.data(), target_pos.data(), {Kp(0,2), Kp(1,2), Kp(2,2)}, {Kd(0,2), Kd(1,2), Kd(2,2)});
                break;
            case 3: // the command to make a robot "run"
                CalculateTorqueRunning(output_torque.data(), target_pos.data(), {Kp(0,1), Kp(1,1), Kp(2,1)}, {Kd(0,1), Kd(1,1), Kd(2,1)});
                break;
            default:
                fill(output_torque.begin(), output_torque.end(), 0.0);
                break;
        }

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
    double _count;
    array<double, 3> previous_command; // 이전 command 값을 저장
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
