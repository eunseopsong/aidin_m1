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

        sub_gains = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Gains_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < 6; ++i) {
                    if (i <= 2)
                        Kp[i]   = msg->data[i];
                    else
                        Kd[i-3] = msg->data[i];
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
    //////////////// Feedforward Control Function ////////////////

    double FeedforwardController(double Kp, double Kd, double target_pos, int index)
    {
        double output_torque = Kp*(target_pos - joint_pos[index]) + Kd*(0 - joint_vel[index]);
        return output_torque;
    }

    double InverseKinematics3D(double px, double py, double pz, double l2, double l3, int case_)
    {
        double th1, th2, th3;
        th2 = 2;
        th1 = fabs(atan2(py, px) - atan2(px, -py)) - M_PI_2;
        // th1 = (atan2(py, px) - atan2(d1, -py)) + M_PI_2;

        double D = (pow(py, 2) + pow(pz, 2) - pow(l2, 2) - pow(l3, 2)) / 2*l2*l3;

        th3 = acos(D);

        if (case_ == 1)
            return th1;
        else if (case_ == 2)
            return th2;
        else if (case_ == 3)
            return th3;
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
        // double yVal = 0.095*cos(joint_pos[0]);  // add this !
        double yVal = 95;
        double xVal, zVal;
        SplineTrajectory(t, T, xVal, zVal);

        double xVal_counter, zVal_counter;
        SplineTrajectory(t_counter, T, xVal_counter, zVal_counter);

        // Calulate the target_pos using Inverse Kinematics
        double target_pos[12];

        target_pos[0] = InverseKinematics3D(yVal, zVal, xVal, 250, 250, 1);
        target_pos[1] = InverseKinematics2D(xVal, zVal, 1);
        target_pos[2] = InverseKinematics2D(xVal, zVal, 2);
        double check = InverseKinematics3D(yVal, zVal, xVal, 250, 250, 3);

        // target_pos[0] = InverseKinematics3D(95, -300, 100, 95, 250, 250, 1);
        // target_pos[1] = InverseKinematics3D(95, -300, 100, 95, 250, 250, 2);
        // target_pos[2] = InverseKinematics3D(95, -300, 100, 95, 250, 250, 3);

        // target_pos[0] = Inverse_K(95, zVal, xVal, true, 95, 250, 250, 1);
        // target_pos[1] = Inverse_K(95, zVal, xVal, true, 95, 250, 250, 2);
        // target_pos[2] = Inverse_K(95, zVal, xVal, true, 95, 250, 250, 3);

        target_pos[3] = -angle[0];
        target_pos[4] =  InverseKinematics2D(xVal_counter, zVal_counter, 1);
        target_pos[5] =  InverseKinematics2D(xVal_counter, zVal_counter, 2);

        target_pos[6] = -target_pos[3];
        target_pos[7] =  target_pos[4];
        target_pos[8] =  target_pos[5];

        target_pos[9]  = -target_pos[0];
        target_pos[10] =  target_pos[1];
        target_pos[11] =  target_pos[2];

        // Calculate the output_torque using PD control
        double output_torque[12];

        for (int i=0; i<12; i++){
            if (i<3) {
                output_torque[i] = PDController(Kp[i],   Kd[i],   target_pos[i], joint_pos[i], joint_vel[i]);
            } else if (i<6) {
                output_torque[i] = PDController(Kp[i-3], Kd[i-3], target_pos[i], joint_pos[i], joint_vel[i]);
            } else if (i<9) {
                if (i == 6)
                    output_torque[i] = -output_torque[i-3];
                else
                    output_torque[i] =  output_torque[i-3];
            } else {
                if (i == 9)
                    output_torque[i] = -output_torque[i-9];
                else
                    output_torque[i] =  output_torque[i-9];
            }
        }
        /////////////// Publish Desired Pose ///////////////
        std_msgs::msg::Float32MultiArray targetpos_msg;
        targetpos_msg.data.clear();

        for (int i=0; i < 12; i++){
            targetpos_msg.data.push_back(check);
        }

        pub_targetpos->publish(targetpos_msg);

        ////////////////// Publish Torque //////////////////
        std_msgs::msg::Float32MultiArray torque_msg;
        torque_msg.data.clear();

        for (int i=0; i < 12; i++){
            torque_msg.data.push_back(output_torque[i]);
        }

        pub_torque->publish(torque_msg);
    }
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointpos;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointvel;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_angles;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_gains;

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
