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

    double FeedforwardController(double Kp, double Kd, double th[3], int case_)
    {   // 2024.03.08.13:56 torque publish가 먹통 되는 문제 발생 -> why..?
        Matrix3d M, C, B;   // 3x3 행렬

        Vector3d PD;  // 크기 3의 벡터
        Vector3d joint_square;
        Vector3d joint_multiple;

        Vector3d G, T;

        // double m1  = 12.172;
        double m2  = 1.688, m3  = 1.688;
        double L1  = 0.095, L2  = 0.250;
        // double L3  = 0.250;
        // double Lg1 = 0.060;
        double Lg2 = 0.125, Lg3 = 0.125;

        double PD_term_1 = Kp*(th[0] - joint_pos[3]) + Kd*(0 - joint_vel[3]);
        double PD_term_2 = Kp*(th[1] - joint_pos[4]) + Kd*(0 - joint_vel[4]);
        double PD_term_3 = Kp*(th[2] - joint_pos[5]) + Kd*(0 - joint_vel[5]);

        M <<  L1*pow(m2,2) + L1*pow(m3,2) + 10697018289/90071992547, -L1*cos(th[0])*(L2*m3*cos(th[1]) - Lg3*m3*sin(th[1] + th[2]) + Lg2*m2*cos(th[1])), L1*Lg3*m3*sin(th[1] + th[2])*cos(th[0]),
             -L1*cos(th[0])*(L2*m3*cos(th[1]) - Lg3*m3*sin(th[1] + th[2]) + Lg2*m2*cos(th[1])), m3*pow(L2,2) - 2*m3*sin(th[2])*L2*Lg3 + m2*pow(Lg2,2) + m3*pow(Lg3,2) + 63965189/900719925, m3*pow(Lg3,2) - L2*m3*sin(th[2])*Lg3 + 63965189/900719925,
              L1*Lg3*m3*sin(th[1] + th[2])*cos(th[0]), m3*pow(Lg3,2) - L2*m3*sin(th[2])*Lg3 + 63965189/1801439850, m3*pow(Lg3,2) + 63965189/1801439850;
        PD << PD_term_1, PD_term_2, PD_term_3;

        C <<  0, L1*cos(th[0])*(L2*m3*sin(th[1]) + Lg2*m2*sin(th[1]) + Lg3*m3*cos(th[1] + th[2])), L1*Lg3*m3*cos(th[1] + th[2])*cos(th[0]),
              L1*sin(th[0])*(L2*m3*cos(th[1]) - Lg3*m3*sin(th[1] + th[2]) + Lg2*m2*cos(th[1])), 0, -L2*Lg3*m3*cos(th[2]),
             -L1*Lg3*m3*sin(th[1] + th[2])*sin(th[0]), L2*Lg3*m3*cos(th[2]), 0;
        joint_square << pow(joint_vel[3], 2), pow(joint_vel[4], 2), pow(joint_vel[5], 2);

        B << 0, 0,  L1*Lg3*m3*cos(th[1] + th[2])*cos(th[0]),
             0, 0, -L2*Lg3*m3*cos(th[2]),
             0, 0,  0;
        joint_multiple << joint_vel[3]*joint_vel[4], joint_vel[3]*joint_vel[5], joint_vel[4]*joint_vel[5];

        G << (981*L1*sin(th[0])*(m2 + m3))/100 - (981*L1*m2*cos(th[0]))/100,
             (981*L2*m3*cos(th[1]))/100 - (981*Lg3*m3*sin(th[1] + th[2]))/50 - (981*L1*m3*cos(th[0]))/100 - (981*Lg2*m2*sin(th[1]))/100 + (981*Lg2*m2*cos(th[1]))/100,
            -(981*m3*(Lg3*cos(th[1] + th[2]) + L2*sin(th[1])))/100 - (981*Lg3*m3*cos(th[1] + th[2]))/100;

        T = M*PD + C*joint_square + B*joint_multiple + G;

        if (case_ == 0){
            return T[0];
        } else if (case_ == 1) {
            return T[1];
        } else {
            return T[2];
        }
    }

    /////////////// timer_에 의해 호출되어 Publish를 실행하는 함수 ///////////////

    void CalculateAndPublishTorque()
    {
        // Initialization
        count_ = count_ + 0.001; // CalculateAndPublishTorque가 실행될 때마다 count_ = count + 1ms; -> count_ 는 실제 시뮬레이션 시간을 나타내는 변수가 됨

        double vel_of_body = 500;    // The target velocity of whole robot bodys
        double T = 1.6;               // The period of the whole trajectory phase
        double t = fmod(count_, T);
        double t_counter = fmod(count_ + 0.800 , T);

        // Calculate the coordinate using Trajectory Function
        double yVal = 95;
        double xVal, zVal;
        SplineTrajectory(t, T, vel_of_body, xVal, zVal);

        double xVal_counter, zVal_counter;
        SplineTrajectory(t_counter, T, vel_of_body, xVal_counter, zVal_counter);

        // Calulate the target_pos using Inverse Kinematics
        double target_pos[12];
        double LF_target_pos[3];
        double RF_target_pos[3];

        InverseKinematics3D(yVal, zVal, xVal, yVal, 250, 250, LF_target_pos);
        InverseKinematics3D(yVal, zVal_counter, xVal_counter, yVal, 250, 250, RF_target_pos);
        for (int i=1; i<6; i++) {
            if (i<3)
                target_pos[i] = LF_target_pos[i];
            else
                target_pos[i] = RF_target_pos[i-3];
        }

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
                // output_torque[i] = PDController(Kp[i],   Kd[i],   target_pos[i], joint_pos[i], joint_vel[i]);
                output_torque[i] =0;
            } else if (i<6) {
                // output_torque[i] = PDController(Kp[i-3], Kd[i-3], target_pos[i], joint_pos[i], joint_vel[i]);
                output_torque[i] = FeedforwardController(Kp[i-3], Kd[i-3], RF_target_pos, i-3);
            } else if (i<9) {
                if (i == 6)
                    // output_torque[i] = -output_torque[i-3];
                    output_torque[i] =0;
                else
                    // output_torque[i] =  output_torque[i-3];
                    output_torque[i] =0;
            } else {
                if (i == 9)
                    // output_torque[i] = -output_torque[i-9];
                    output_torque[i] =0;
                else
                    // output_torque[i] =  output_torque[i-9];
                    output_torque[i] =0;
            }
        }

        /////////////// Publish Desired Pose ///////////////
        std_msgs::msg::Float32MultiArray targetpos_msg;
        targetpos_msg.data.clear();

        for (int i=0; i < 12; i++){
            targetpos_msg.data.push_back(target_pos[i]);
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
