#include <iostream>
#include <cmath>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std;

class JointControl: public rclcpp::Node
{
public:
    JointControl() : Node("joint_control_node")
    {
        // Subscribe to JointPos_sim and JointVel_sim topics
        sub_jointpos = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/leg/JointPos_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                joint1_pos = msg->data[0];
                joint2_pos = msg->data[1];
                joint3_pos = msg->data[2];
                CalculateAndPublishTorque();
            });

        sub_jointvel = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/leg/JointVel_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                joint1_vel = msg->data[0];
                joint2_vel = msg->data[1];
                joint3_vel = msg->data[2];
                CalculateAndPublishTorque();
            });

        sub_simtime = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", rclcpp::QoS(10).best_effort(),
            [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
                sim_time = msg->clock.sec + (msg->clock.nanosec)/1000000000.0;
                CalculateAndPublishTorque();
            });

        pub_torque = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/leg/Torque_sim", 10);

        sub_gains = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/leg/Gains_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                kp[0] = msg->data[0];
                kp[1] = msg->data[1];
                kp[2] = msg->data[2];
                kd[0] = msg->data[3];
                kd[1] = msg->data[4];
                kd[2] = msg->data[5];
            });


        // Publish desired joint poses
        pub_desiredpos = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/leg/DesiredPos", 10);
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointpos;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointvel;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_simtime;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_torque;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_gains;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_desiredpos;

    float joint1_pos, joint2_pos, joint3_pos, joint1_vel, joint2_vel, joint3_vel;   // Joint angle & velocity
    double sim_time;                    // Gazebo simulation time
    double q1, q2, q3, l1, l2, l3;      // initial leg angle, leg length
    double kp[3], kd[3];                // Gains


    // Calculation of Bezier Curve
    double factorial(int n) {
        if (n <= 1) {
            return 1;
        } else {
            return n * factorial(n-1);
        }
    }

    double nchoosek(int n, int k) {
        return factorial(n) / (factorial(k) * factorial(n-k));
    }

    void bezierEndpoint(double t, double &y, double &z) {
        q1 = 0, q2 = 40 * M_PI / 180 - M_PI, q3 = 90 * M_PI / 180;          // initial leg angle
        l1 = 80, l2 = 230, l3 = 230;        // leg length
        double S = 0.2, V = 1.8*1000;       // S: period of gate cycle, V: gate velocity
        double D = S*V/2;
        double pe[2];

        // Points for Making Bezier Curve
        // pe[0] = l3*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + l1*cos(q1) + l2*sin(q1)*sin(q2);    // x forward kinematics
        pe[0] = l3*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) + l2*cos(q2);                                            // y forward kinematics
        pe[1] = l3*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - l1*sin(q1) + l2*cos(q1)*sin(q2);       // z forward kinematics

        double P0[2], P1[2], P2[2], P3[2], P4[2], P5[2], P6[2], P7[2], P8[2];
        P0[0] = pe[0];              P0[1] = pe[1];
        P1[0] = pe[0]-D;            P1[1] = pe[1];
        P2[0] = pe[0]-D-V/10*S;     P2[1] = pe[1];
        P3[0] = pe[0]-D-V/10*S-10;  P3[1] = 7*pe[1]/8;
        P4[0] = pe[0];              P4[1] = 7*pe[1]/8;
        P5[0] = pe[0];              P5[1] = 19*pe[1]/24;
        P6[0] = pe[0]+D+V/10*S+10;  P6[1] = 5*pe[1]/6;
        P7[0] = pe[0]+D+V/10*S;     P7[1] = pe[1];
        P8[0] = pe[0]+D;            P8[1] = pe[1];


        double By = 0, Bz = 0;
        t = fmod(t, 2*S);

        if (t <= S/2) {
            double p[2][2] = {{P0[0], P0[1]}, {P1[0], P1[1]}};
            int n = 2, n1 = n - 1;

            for (int i=0; i<=n1; i++) {
                By += nchoosek(n1, i) * p[i][0] * pow(1 - (t / (S / 2)), n1 - i) * pow(t / (S / 2), i);
                Bz += nchoosek(n1, i) * p[i][1] * pow(1 - (t / (S / 2)), n1 - i) * pow(t / (S / 2), i);
            }
        }
        else if (t <= 3*S/2) {
            double p[11][2] = {{P1[0], P1[1]}, {P2[0], P2[1]}, {P3[0], P3[1]}, {P4[0], P4[1]},
                                {P5[0], P5[1]}, {P6[0], P6[1]}, {P7[0], P7[1]}, {P8[0], P8[1]}};
            int n = 11, n1 = n - 1;

            for (int i = 0; i <= n1; i++) {
                By += nchoosek(n1, i) * p[i][0] * pow(1 - ((t - (S / 2)) / S), n1 - i) * pow((t - (S / 2)) / S, i);
                Bz += nchoosek(n1, i) * p[i][1] * pow(1 - ((t - (S / 2)) / S), n1 - i) * pow((t - (S / 2)) / S, i);
            }
        }
        else {
            double p[2][2] = {{P8[0], P8[1]}, {P0[0], P0[1]}};
            int n = 2, n1 = n - 1;

            for (int i = 0; i <= n1; i++) {
                By += nchoosek(n1, i) * p[i][0] * pow(1 - ((t - (3 * S / 2)) / (S / 2)), n1 - i) * pow((t - (3 * S / 2)) / (S / 2), i);
                Bz += nchoosek(n1, i) * p[i][1] * pow(1 - ((t - (3 * S / 2)) / (S / 2)), n1 - i) * pow((t - (3 * S / 2)) / (S / 2), i);
            }
        }

        y = By;
        z = Bz;
    }

    void CalculateAndPublishTorque()
    {
        // 발 끝 좌표
        double y, z;
        bezierEndpoint(sim_time, y, z);
        double x = 79;

        // 조인트 각도 계산
        double costh1 = (x*l1 + sqrt(x*x*l1*l1 - (x*x + z*z)*(l1*l1 - z*z))) / (x*x + z*z);

        if (costh1 >= -1 && costh1 <= 1)
        {
            double th1 = atan2(sqrt(1 - costh1*costh1), costh1);

            double p_rot_y = y*cos(th1) - z*sin(th1);
            double p_rot_z = y*sin(th1) + z*cos(th1);

            double th = atan2(p_rot_z, p_rot_y);
            double l = sqrt(p_rot_y*p_rot_y + p_rot_z*p_rot_z);

            double th2 = M_PI + th - acos((l2*l2 + l*l - l3*l3) / (2*l2*l));
            double th3 = M_PI - acos((l2*l2 + l3*l3 - l*l) / (2*l2*l3));


            std_msgs::msg::Float32MultiArray torque_msg;
            torque_msg.data.clear();

            torque_msg.data.push_back(kp[0]*(th1 - joint1_pos) + kd[0]*(0-joint1_vel));
            torque_msg.data.push_back(kp[1]*(th2-M_PI/2 - joint2_pos) + kd[1]*(0-joint2_vel));
            torque_msg.data.push_back(kp[2]*(th3 - joint3_pos) + kd[2]*(0-joint3_vel));

            pub_torque->publish(torque_msg);


            std_msgs::msg::Float32MultiArray desiredpos_msg;
            desiredpos_msg.data.clear();

            desiredpos_msg.data.push_back(th1);
            desiredpos_msg.data.push_back(th2-M_PI/2);
            desiredpos_msg.data.push_back(th3);

            pub_desiredpos->publish(desiredpos_msg);
        }
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointControl>());
    rclcpp::shutdown();

    return 0;
}
