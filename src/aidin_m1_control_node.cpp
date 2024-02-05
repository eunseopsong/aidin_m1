#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std;
using namespace Eigen;

class JointControl: public rclcpp::Node
{
public:
    JointControl() : Node("aidin_m1_control_node")
    {
        // Subscribe to JointPos_sim and JointVel_sim topics
        sub_jointpos = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/JointPos_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                joint1_pos = msg->data[0];
                joint2_pos = msg->data[1];
                joint3_pos = msg->data[2];
                joint4_pos = msg->data[3];
                joint5_pos = msg->data[4];
                joint6_pos = msg->data[5];
                joint7_pos = msg->data[6];
                joint8_pos = msg->data[7];
                joint9_pos = msg->data[8];
                joint10_pos = msg->data[9];
                joint11_pos = msg->data[10];
                joint12_pos = msg->data[11];
                CalculateAndPublishTorque();
            });

        sub_jointvel = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/JointVel_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                joint1_vel = msg->data[0];
                joint2_vel = msg->data[1];
                joint3_vel = msg->data[2];
                joint4_vel = msg->data[3];
                joint5_vel = msg->data[4];
                joint6_vel = msg->data[5];
                joint7_vel = msg->data[6];
                joint8_vel = msg->data[7];
                joint9_vel = msg->data[8];
                joint10_vel = msg->data[9];
                joint11_vel = msg->data[10];
                joint12_vel = msg->data[11];
                CalculateAndPublishTorque();
            });

        sub_simtime = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", rclcpp::QoS(10).best_effort(),
            [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
                sim_time = msg->clock.sec + (msg->clock.nanosec)/1000000000.0;
                CalculateAndPublishTorque();
            });

        pub_torque = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Torque_sim", 10);


        sub_gains = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Gains_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                kp[0] = msg->data[0];
                kp[1] = msg->data[1];
                kp[2] = msg->data[2];
                kd[0] = msg->data[3];
                kd[1] = msg->data[4];
                kd[2] = msg->data[5];
            });

        sub_angles = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/Angles_sim", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                ag[0] = msg->data[0];
                ag[1] = msg->data[1];
            });


        // Publish desired joint poses
        pub_desiredpos = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/aidin_m1/DesiredPos", 10);
    }

private:
    /////////////////// Initializing ////////////////////
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointpos;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointvel;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_simtime;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_torque;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_gains;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_angles;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_desiredpos;

    // Joint angle
    float joint1_pos, joint2_pos, joint3_pos, joint4_pos, joint5_pos, joint6_pos, joint7_pos, joint8_pos, joint9_pos, joint10_pos, joint11_pos, joint12_pos;
    // Joint velocity
    float joint1_vel, joint2_vel, joint3_vel, joint4_vel, joint5_vel, joint6_vel, joint7_vel, joint8_vel, joint9_vel, joint10_vel, joint11_vel, joint12_vel;

    double sim_time;                    // Gazebo simulation time
    double ag[3];                       // Angles
    double kp[3], kd[3];                // Gains

    //////////// Method of Undetermined Coefficients using Eigen ////////////

    void solve(double d, double e, double f, double T, double singularity, double B_val[], double arr[6])
    {
        Matrix3d A;  // 3x3 행렬
        Vector3d B;  // 크기 3의 벡터

        // 행렬과 벡터 값 설정 (A는 주어진 행렬, B는 상수 벡터)
        A << (5*pow(singularity, 4)), (4*pow(singularity, 3)), (3*pow(singularity, 2)), pow((T/4), 5), pow((T/4), 4), pow((T/4), 3), 20*pow((T/4), 3), 12*pow((T/4), 2), 6*pow((T/4), 1);
        B << B_val[0], B_val[1], B_val[2];

        // 선형 시스템 풀기
        Vector3d solution = A.colPivHouseholderQr().solve(B);

        // 결과값 저장
        double S[6] = {solution[0], solution[1], solution[2], d, e, f};

        // 결과값 반환
        for (int i=0; i < 6; i++)
            arr[i] = S[i];
    }

    ////////////////// for STandingPhase //////////////////
    // 시간에 따른 STandingPhase x 좌표의 변화를 저장하는 함수
    double CalculateXValues(double l, double v, double t)
    {
        double returnXValue = (l/2) - v*t;

        return returnXValue;
    }

    //////////////////// for SWingPhase ////////////////////

double CalculateValues(double S[], double t, double T, int cases)
{
    double returnValue;

    if (cases == 2 || cases == 5) {
        // SWingPhase (x & z)
        returnValue = S[0]*pow(t - T/2, 5) + S[1]*pow(t - T/2, 4) + S[2]*pow(t - T/2, 3) + S[3]*pow(t - T/2, 2) + S[4]*pow(t - T/2, 1) + S[5];
    } else if (cases == 3) {
        // ReversePhase (x)
        returnValue = -S[0]*pow(T-t, 5) - S[1]*pow(T-t, 4) - S[2]*pow(T-t, 3) - S[3]*pow(T-t, 2) - S[4]*pow(T-t, 1) - S[5];
    } else {
        // ReversePhase (z)
        returnValue = S[0]*pow(T-t, 5) + S[1]*pow(T-t, 4) + S[2]*pow(T-t, 3) + S[3]*pow(T-t, 2) + S[4]*pow(T-t, 1) + S[5];
    }

    return returnValue;
}

    ////////////////////////////////////////////////////////
    //////////////////// for Kinematics ////////////////////
    ////////////////////////////////////////////////////////

    double CalculateKinematics(double xVal, double zVal, int cases)
    {
        // double returnDegree;
        double len_hip = 250, len_knee = 250;

        zVal = -zVal;

        // Calculate Knee Joint Value using Inverse Kinematics
        double costh3 = (pow(xVal, 2) + pow(zVal, 2) - pow(len_hip, 2) - pow(len_knee ,2)) / (2*len_hip*len_knee);
        double knee_degree = acos(costh3);

        // Calculate Hip Joint Value using Inverse Kinematics
        double hip_degree = atan2(zVal, xVal) - atan2(len_knee*sin(knee_degree), len_hip + len_knee*cos(knee_degree));

        knee_degree -= M_PI_2;

        if (cases == 1)
            return hip_degree;
        else
            return knee_degree;

        // return returnDegree;
    }

    void SplineTrajectory(double sim_time, double &xVal, double &zVal)
    {
        /////////////////// Initializing ////////////////////

        double vel_of_body = 1600;
        double T = 0.5;
        double length_of_STanding_phase = vel_of_body * T /2;
        double height = 1656/5;

        int SW_x_case = 2, Reverse_x_case = 3;
        int SW_z_case = 5, Reverse_z_case = 6;

        // undetermined coefficients (a1*t^5 + b1*t^4 + c1*t^3 + d1*t^2 + e1*t + f1)
        double d1 = 0, e1 = -(vel_of_body), f1 = -length_of_STanding_phase/2;
        double singular1 = T/16;
        double B_val1[3] = {-e1, -(f1 +e1*T/4), 0};
        double S1[6];

        // undetermined coefficients (a2*t^5 + b2*t^4 + c2*t^3 + d2*t^2 + e2*t + f2)
        double d2 = 0, e2 = 0, f2 = -height;
        double singular2 = T/4;
        double B_val2[3] = {0, -(5*height/6 +f2), 0};
        double S2[6];

        /////////////////////// Calculate the return Value ////////////////////////
        // Initialize the return value
        double returnXValue, returnZValue;
        double t = fmod(sim_time, T);

        if (t <= T/2) {
            returnXValue = CalculateXValues(length_of_STanding_phase, vel_of_body, t);
            returnZValue = -height;

        } else if (t <= T/4*3) {
            // solve undetermined coefficients (double S1[6] = {a1, b1, c1, d1, e1, f1};)
            solve(d1, e1, f1, T, singular1, B_val1, S1);
            returnXValue = CalculateValues(S1, t, T, SW_x_case);

            // solve undetermined coefficients (double S2[6] = {a2, b2, c2, d2, e2, f2};)
            solve(d2, e2, f2, T, singular2, B_val2, S2);
            returnZValue = CalculateValues(S2, t, T, SW_z_case);

        } else {
            returnXValue = CalculateValues(S1, t, T, Reverse_x_case);
            returnZValue = CalculateValues(S2, t, T, Reverse_z_case);
        }
        xVal = returnXValue;
        zVal = returnZValue;
    }

    void CalculateAndPublishTorque()
    {
        // double yVal = 210;
        double xVal, zVal;
        SplineTrajectory(sim_time, xVal, zVal);

        double RF_hip_degree = CalculateKinematics(xVal, zVal, 1);
        double RF_knee_degree = CalculateKinematics(xVal, zVal, 2);

        // double RF_hip_output_torque = PID(kp[1], kd[1], hip_degree, 1);
        // double RF_knee_output_torque = PID(kp[2], kd[2], knee_degree, 2);

        std_msgs::msg::Float32MultiArray torque_msg;
        torque_msg.data.clear();

        torque_msg.data.push_back(ag[0]);
        torque_msg.data.push_back(0);
        torque_msg.data.push_back(0);
        torque_msg.data.push_back(-ag[0]);
        torque_msg.data.push_back(kp[1]*(RF_hip_degree - joint5_pos) + kd[1]*(0-joint5_vel));
        torque_msg.data.push_back(kp[2]*(RF_knee_degree - joint6_pos) + kd[2]*(0-joint6_vel));
        torque_msg.data.push_back(ag[0]);
        torque_msg.data.push_back(0);
        torque_msg.data.push_back(0);
        torque_msg.data.push_back(-ag[0]);
        torque_msg.data.push_back(0);
        torque_msg.data.push_back(0);

        pub_torque->publish(torque_msg);


        std_msgs::msg::Float32MultiArray desiredpos_msg;
        desiredpos_msg.data.clear();

        desiredpos_msg.data.push_back(ag[0]);
        desiredpos_msg.data.push_back(0);
        desiredpos_msg.data.push_back(0);
        desiredpos_msg.data.push_back(ag[0]);
        desiredpos_msg.data.push_back(RF_hip_degree);
        desiredpos_msg.data.push_back(RF_knee_degree);
        desiredpos_msg.data.push_back(ag[0]);
        desiredpos_msg.data.push_back(0);
        desiredpos_msg.data.push_back(0);
        desiredpos_msg.data.push_back(ag[0]);
        desiredpos_msg.data.push_back(0);
        desiredpos_msg.data.push_back(0);

        pub_desiredpos->publish(desiredpos_msg);


    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointControl>());
    rclcpp::shutdown();

    return 0;
}
