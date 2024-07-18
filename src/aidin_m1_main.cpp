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
        // Subscribe
        sub_bodypose = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/BodyPose_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < 6; ++i) {
                    body_pose[i] = msg->data[i];
                }
            });

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

        sub_bodypos = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/BodyPos_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < 3; ++i) {
                    body_pos[i] = msg->data[i];
                }
            });

        sub_bodyvel = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/BodyVel_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < 3; ++i) {
                    body_vel[i] = msg->data[i];
                }
            });

        sub_imu = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/IMU_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < 9; ++i) {
                    imu[i] = msg->data[i];
                }
            });

        sub_contact = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Contact_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                for (int i = 0; i < 4; ++i) {
                    contact[i] = msg->data[i];
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
        double LB_target_pos[3];
        double RB_target_pos[3];

        InverseKinematics3D(yVal, zVal, xVal, yVal, 250, 250, LF_target_pos);
        InverseKinematics3D(yVal, zVal_counter, xVal_counter, yVal, 250, 250, RF_target_pos);
        InverseKinematics3D(yVal, zVal_counter, xVal_counter, yVal, 250, 250, LB_target_pos);
        InverseKinematics3D(yVal, zVal, xVal, yVal, 250, 250, RB_target_pos);

        for (int i=1; i<6; i++)
        {
            if (i<3)
                target_pos[i] = LF_target_pos[i];
            else if (i<6)
                target_pos[i] = RF_target_pos[i-3];
            else if (i<9)
                target_pos[i] = LB_target_pos[i-6];
            else
                target_pos[i] = RB_target_pos[i-9];
        }

        // Calculate the output_torque using PD or Feedforward control
        double output_torque[12];

        for (int i=0; i<12; i++)
        {

            if (Kp[0] > 0 && Kp[1] == 0)
            {
                if (i==0)      // LF joint
                {
                    // output_torque[i] = PDController(Kp[i],   Kd[i],   target_pos[i], joint_pos[i], joint_vel[i]);
                    output_torque[i] = FeedforwardController(Kp[i], Kd[i], LF_target_pos, i, 0);
                }
                else if (i==3) // RF joint
                {
                    // output_torque[i] = PDController(Kp[i-3], Kd[i-3], target_pos[i], joint_pos[i], joint_vel[i]);
                    output_torque[i] = FeedforwardController(Kp[i-3], Kd[i-3], RF_target_pos, i-3, 3);
                }
                else if (i==6) // LB joint
                {
                    output_torque[i] = FeedforwardController(Kp[i-6], Kd[i-6], LB_target_pos, i-6, 6);
                }
                else if (i==9) // RB joint
                {
                    output_torque[i] = FeedforwardController(Kp[i-9], Kd[i-9], RB_target_pos, i-9, 9);
                }
                else
                    output_torque[i] = 0;
            }
            else if (Kp[0] > 0 && Kp[1] > 0)
            {
                if (i<3)      // LF joint
                {
                    // output_torque[i] = PDController(Kp[i],   Kd[i],   target_pos[i], joint_pos[i], joint_vel[i]);
                    output_torque[i] = FeedforwardController(Kp[i], Kd[i], LF_target_pos, i, 0);
                }
                else if (i<6) // RF joint
                {
                    // output_torque[i] = PDController(Kp[i-3], Kd[i-3], target_pos[i], joint_pos[i], joint_vel[i]);
                    output_torque[i] = FeedforwardController(Kp[i-3], Kd[i-3], RF_target_pos, i-3, 3);
                }
                else if (i<9) // LB joint
                {
                    output_torque[i] = FeedforwardController(Kp[i-6], Kd[i-6], LB_target_pos, i-6, 6);
                }
                else          // RB joint
                {
                    output_torque[i] = FeedforwardController(Kp[i-9], Kd[i-9], RB_target_pos, i-9, 9);
                }
            }
            else
                output_torque[i] = 0;
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
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_bodypose;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointpos;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointvel;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_bodypos;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_bodyvel;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_imu;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_contact;

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
