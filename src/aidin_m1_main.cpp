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
    {
        double PD_term_1 = Kp*(th[0] - joint_pos[3]) + Kd*(0 - joint_vel[3]);
        double PD_term_2 = Kp*(th[1] - joint_pos[4]) + Kd*(0 - joint_vel[4]);
        double PD_term_3 = Kp*(th[2] - joint_pos[5]) + Kd*(0 - joint_vel[5]);

        double M11 = (2527*cos(th[0]))/160000 + 10031349019590/46116860184273;
        double M12 = (19*cos(th[1])*sin(th[2]))/3200 - (57*cos(th[1]))/3200 + (19*cos(th[2])*sin(th[1]))/3200 - 3890049390/5902958103587056;
        double M13 = (19*cos(th[1])*sin(th[2]))/3200 + (19*cos(th[2])*sin(th[1]))/3200 - 3890049390/5902958103587056;
        
        double M21 = - (57*cos(th[0]))/3200 - 3890049390/5902958103587056;
        double M22 = cos(th[1] + th[2])/128 + (5*cos(th[1]))/128 - sin(th[1] + th[2])/64 + 4459359147/2882303761517;
        double M23 = cos(th[1] + th[2])/128 - sin(th[1] + th[2])/64 + 4459359147/2882303761517;
        double M31 = - 38900493902/118059162071741130;
        double M32 = cos(th[1] + th[2])/128 - sin(-th[1])/64 + 4459359147/5764607523034;
        double M33 =  cos(th[1] + th[2])/128 + 4459359146/5764607523034;

        double C11 = 0;
        double C12 = (19*(cos(th[1] + th[2]) + 3*sin(th[1])))/3200;
        double C13 = (19*cos(th[1] + th[2]))/3200;
        
        double C21 = (57*sin(th[0]))/3200;
        double C22 = 0;
        double C23 = -cos(th[1] + th[2])/64;
        double C31 = 0;
        double C32 = cos(-th[1])/128 + cos(th[1] + th[2])/128;
        double C33 = 0;

        double B11 = 0;
        double B12 = 0;
        double B13 =  (19*cos(th[1] + th[2]))/1600;
        
        double B21 = -(57*sin(th[1]))/3200 - (19*cos(th[1])*cos(th[2]))/3200 + (19*sin(th[1])*sin(th[2]))/3200 + (19*cos(th[0]))/3200;
        double B22 =  (19*sin(th[1])*sin(th[2]))/3200 - (19*cos(th[1])*cos(th[2]))/3200 + (19*cos(th[0]))/3200;
        double B23 = -cos(-th[1])/64 - cos(th[1] + th[2])/64;
        double B31 =  (19*sin(th[1])*sin(th[2]))/3200 - (19*cos(th[1])*cos(th[2]))/3200 + (19*cos(th[0]))/3200;
        double B32 =  (19*sin(th[1])*sin(th[2]))/3200 - (19*cos(th[1])*cos(th[2]))/3200 + (19*cos(th[0]))/3200;
        double B33 =  cos(th[1] + th[2])/64 - cos(-th[1])/64;

        double U1 = (55917*sin(th[0]))/20000 + 8829/500;
        
        double U2 = (18639*sin(th[0]))/20000 - (981*sin(th[1]))/800 + 2943/1000;
        double U3 = (18639*sin(th[0]))/20000 - (981*sin(th[1] + th[2] + M_PI/2))/800 - (981*sin(th[1]))/400 + 2943/1000;

        double scap_output_torque = (M11+M21+M31)*PD_term_1 + (C11+C21+C31)*joint_vel[0] + 2*(B11+B21+B31)*joint_vel[0]*joint_vel[1] + U1;
        double hip_output_torque  = (M12+M22+M32)*PD_term_2 + (C12+C22+C32)*joint_vel[1] + 2*(B12+B22+B32)*joint_vel[1]*joint_vel[2] + U2;
        double knee_output_torque = (M13+M23+M33)*PD_term_3 + (C13+C23+C33)*joint_vel[2] + 2*(B13+B23+B33)*joint_vel[0]*joint_vel[2] + U3;

        if (case_ == 0){
            return scap_output_torque;
        } else if (case_ == 1) {
            return hip_output_torque;
        } else if (case_ == 2) {
            return knee_output_torque;
        }
    }

    /////////////// timer_에 의해 호출되어 Publish를 실행하는 함수 ///////////////

    void CalculateAndPublishTorque()
    {
        // Initialization
        count_ = count_ + 0.001; // CalculateAndPublishTorque가 실행될 때마다 count_ = count + 1ms; -> count_ 는 실제 시뮬레이션 시간을 나타내는 변수가 됨

        double vel_of_body = 1000;    // The target velocity of whole robot bodys
        double T = 0.8;               // The period of the whole trajectory phase
        double t = fmod(count_, T);
        double t_counter = fmod(count_ + 0.400 , T);

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
                output_torque[i] = PDController(Kp[i],   Kd[i],   target_pos[i], joint_pos[i], joint_vel[i]);
            } else if (i<6) {
                if (i==5)
                    output_torque[i] = FeedforwardController(Kp[i-3], Kd[i-3], RF_target_pos, i-3);
                else
                    output_torque[i] = PDController(Kp[i-3],   Kd[i-3],   target_pos[i], joint_pos[i], joint_vel[i]);
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
