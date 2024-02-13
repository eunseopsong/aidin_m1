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

        // Publish torque & desired joint poses
        pub_torque = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Torque_sim", 10);

        pub_desiredpos = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/DesiredPos", 10);

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

    double PID(double kp, double kd, double taret_pos, int index)
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
        double LF_scap_degree = angle[0];
        double LF_hip_degree  = InverseKinematics(xVal, zVal, 1);
        double LF_knee_degree = InverseKinematics(xVal, zVal, 2);

        double RF_scap_degree = -angle[0];
        double RF_hip_degree  = InverseKinematics(xVal_counter, zVal_counter, 1);
        double RF_knee_degree = InverseKinematics(xVal_counter, zVal_counter, 2);

        double LB_scap_degree = -RF_scap_degree;
        double LB_hip_degree  =  RF_hip_degree;
        double LB_knee_degree =  RF_knee_degree;

        double RB_scap_degree = -LF_scap_degree;
        double RB_hip_degree  =  LF_hip_degree;
        double RB_knee_degree =  LF_knee_degree;

        // Calculate the output_torque using PD control
        double LF_scap_output_torque = PID(kp[0], kd[0], LF_scap_degree, 0);
        double LF_hip_output_torque  = PID(kp[1], kd[1], LF_hip_degree,  1);
        double LF_knee_output_torque = PID(kp[2], kd[2], LF_knee_degree, 2);;

        double RF_scap_output_torque = PID(kp[0], kd[0], RF_scap_degree, 3);
        double RF_hip_output_torque  = PID(kp[1], kd[1], RF_hip_degree,  4);
        double RF_knee_output_torque = PID(kp[2], kd[2], RF_knee_degree, 5);;

        double LB_scap_output_torque = -RF_scap_output_torque;
        double LB_hip_output_torque  =  RF_hip_output_torque;
        double LB_knee_output_torque =  RF_knee_output_torque;

        double RB_scap_output_torque = -LF_scap_output_torque;
        double RB_hip_output_torque  =  LF_hip_output_torque;
        double RB_knee_output_torque =  LF_knee_output_torque;

        ////////////////// Publish Torque //////////////////
        std_msgs::msg::Float32MultiArray torque_msg;
        torque_msg.data.clear();

        torque_msg.data.push_back(LF_scap_output_torque);
        torque_msg.data.push_back(LF_hip_output_torque);
        torque_msg.data.push_back(LF_knee_output_torque);

        torque_msg.data.push_back(RF_scap_output_torque);
        torque_msg.data.push_back(RF_hip_output_torque);
        torque_msg.data.push_back(RF_knee_output_torque);

        torque_msg.data.push_back(LB_scap_output_torque);
        torque_msg.data.push_back(LB_hip_output_torque);
        torque_msg.data.push_back(LB_knee_output_torque);

        torque_msg.data.push_back(RB_scap_output_torque);
        torque_msg.data.push_back(RB_hip_output_torque);
        torque_msg.data.push_back(RB_knee_output_torque);

        pub_torque->publish(torque_msg);

        /////////////// Publish Desired Pose ///////////////
        std_msgs::msg::Float32MultiArray desiredpos_msg;
        desiredpos_msg.data.clear();

        desiredpos_msg.data.push_back(LF_scap_degree);
        desiredpos_msg.data.push_back(LF_hip_degree);
        desiredpos_msg.data.push_back(LF_knee_degree);

        desiredpos_msg.data.push_back(RF_scap_degree);
        desiredpos_msg.data.push_back(RF_hip_degree);
        desiredpos_msg.data.push_back(RF_knee_degree);

        desiredpos_msg.data.push_back(LB_scap_degree);
        desiredpos_msg.data.push_back(LB_hip_degree);
        desiredpos_msg.data.push_back(LB_knee_degree);

        desiredpos_msg.data.push_back(RB_scap_degree);
        desiredpos_msg.data.push_back(RB_hip_degree);
        desiredpos_msg.data.push_back(RB_knee_degree);

        pub_desiredpos->publish(desiredpos_msg);
    }
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointpos;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointvel;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_gains;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_angles;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_simtime;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_torque;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_desiredpos;

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
