#include "func.cpp"

using namespace std;
using namespace Eigen;
using namespace std::chrono_literals;

class JointControl: public rclcpp::Node
{
public:
    JointControl()
    : Node("aidin_m1_control_node"), count_(0) // count_ == 0 (Initialize)
    {
        // Subscribe to JointPos_sim and JointVel_sim topics
        sub_jointpos = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/JointPos_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < 12; ++i) {
                    joint_pos[i] = msg->data[i];
                }
            });

        sub_jointvel = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/JointVel_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < 12; ++i) {
                    joint_vel[i] = msg->data[i];
                }
            });

        sub_simtime = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", rclcpp::QoS(10).best_effort(),
            [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
                sim_time = msg->clock.sec + (msg->clock.nanosec)/1000000000.0;
            });

        sub_gains = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Gains_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < 6; ++i) {
                    if (i <= 2)
                        kp[i]   = msg->data[i];
                    else
                        kd[i-3] = msg->data[i];
                }
            });

        sub_angles = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Angles_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < 3; ++i) {
                    angle[i] = msg->data[i];
                }
            });

        // Publish torque & target joint poses
        pub_torque = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Torque_sim", 10);

        pub_targetpos = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/TargetPos", 10);

        // node Publish 실행 주기 설정 (1ms)
        timer_ = this->create_wall_timer(
            1ms, std::bind(&JointControl::CalculateAndPublishTorque, this));
    }

private:
    /////////////////////// Initializing ////////////////////////
    float joint_pos[12];     // Joint Pose
    float joint_vel[12];     // Joint Velocity

    double sim_time;         // Gazebo simulation time
    double angle[3];         // Angles
    double kp[3], kd[3];     // Gains

    //////////////////// PID Control Function ////////////////////

    double PID(double kp, double kd, double target_pos, int index)
    {
        double output_torque = kp*(target_pos - joint_pos[index]) + kd*(0 - joint_vel[index]);
        return output_torque;
    }

    /////////////// timer_에 의해 호출되어 Publish를 실행하는 함수 ///////////////

    void CalculateAndPublishTorque()
    {
        // Initializing
        count_ = count_ + 0.001; // CalculateAndPublishTorque가 실행될 때마다 count_ = count + 1ms; -> count_ 는 실제 시뮬레이션 시간을 나타내는 변수가 됨
        double T = 0.4;          // The period of the whole trajectory phase
        double t = fmod(count_, T);
        double t_counter = fmod(count_ + 0.200 , T);

        // Calculate the coordinate using Trajectory Function
        double xVal, zVal;
        SplineTrajectory(t, T, xVal, zVal);

        double xVal_counter, zVal_counter;
        SplineTrajectory(t_counter, T, xVal_counter, zVal_counter);

        // Calulate the degree using Inverse Kinematics
        double target_pos[12];

        target_pos[0] = angle[0];
        target_pos[1] = InverseKinematics(xVal, zVal, 1);
        target_pos[2] = InverseKinematics(xVal, zVal, 2);

        target_pos[3] = -angle[0];
        target_pos[4] =  InverseKinematics(xVal_counter, zVal_counter, 1);
        target_pos[5] =  InverseKinematics(xVal_counter, zVal_counter, 2);

        target_pos[6] = -target_pos[3];
        target_pos[7] =  target_pos[4];
        target_pos[8] =  target_pos[5];

        target_pos[9]  = -target_pos[0];
        target_pos[10] =  target_pos[1];
        target_pos[11] =  target_pos[2];

        // Calculate the output_torque using PD control
        double output_torque[12];

        output_torque[0] = PID(kp[0], kd[0], target_pos[0], 0);
        output_torque[1] = PID(kp[1], kd[1], target_pos[1],  1);
        output_torque[2] = PID(kp[2], kd[2], target_pos[2], 2);;

        output_torque[3] = PID(kp[0], kd[0], target_pos[3], 3);
        output_torque[4] = PID(kp[1], kd[1], target_pos[4],  4);
        output_torque[5] = PID(kp[2], kd[2], target_pos[5], 5);;

        output_torque[6] = -output_torque[3];
        output_torque[7] =  output_torque[4];
        output_torque[8] =  output_torque[5];

        output_torque[9]  = -output_torque[0];
        output_torque[10] =  output_torque[1];
        output_torque[11] =  output_torque[2];

        ////////////////// Publish Torque //////////////////
        std_msgs::msg::Float32MultiArray torque_msg;
        torque_msg.data.clear();

        for (int i=0; i < 12; i++){
            torque_msg.data.push_back(output_torque[i]);
        }

        pub_torque->publish(torque_msg);

        /////////////// Publish Desired Pose ///////////////
        std_msgs::msg::Float32MultiArray targetpos_msg;
        targetpos_msg.data.clear();

        for (int i=0; i < 12; i++){
            targetpos_msg.data.push_back(target_pos[i]);
        }

        pub_targetpos->publish(targetpos_msg);
    }
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointpos;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointvel;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_gains;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_angles;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_simtime;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_torque;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_targetpos;

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
