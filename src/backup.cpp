#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cmath>
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std;
using namespace Eigen;
using namespace std::chrono_literals;

#include "func.cpp"

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
                angle[0] = msg->data[0];
                angle[1] = msg->data[1];
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
        double returnXValue = -(l/2) + v*t;

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
        } else if (cases == 6) {
            // ReversePhase (z)
            returnValue = S[0]*pow(T-t, 5) + S[1]*pow(T-t, 4) + S[2]*pow(T-t, 3) + S[3]*pow(T-t, 2) + S[4]*pow(T-t, 1) + S[5];
        }

        return returnValue;
    }

    void SplineTrajectory(double t, double T, double &xVal, double &zVal)
    {
        /////////////////////// Initializing ////////////////////////

        double vel_of_body = 2500;
        double length_of_STanding_phase = vel_of_body * T /2;
        double height = 400;

        // int ST_x_case = 1;
        int SW_x_case = 2, Reverse_x_case = 3;
        // int ST_z_case = 4;
        int SW_z_case = 5, Reverse_z_case = 6;

        // undetermined coefficients of SWxValues (a1*t^5 + b1*t^4 + c1*t^3 + d1*t^2 + e1*t + f1)
        double d1 = 0, e1 = (vel_of_body), f1 = length_of_STanding_phase/2;
        double singular1 = T/16;
        double B_val1[3] = {-e1, -(f1 +e1*T/4), 0};
        double S1[6];

        // undetermined coefficients of SWzValues (a2*t^5 + b2*t^4 + c2*t^3 + d2*t^2 + e2*t + f2)
        double d2 = 0, e2 = 0, f2 = -height;
        double singular2 = T/4;
        double B_val2[3] = {0, -(5*height/6 +f2), 0};
        double S2[6];

        /////////////////////// Calculate the return Value ////////////////////////
        // Initialize the return value
        double returnXValue, returnZValue;

        if (t <= T/2) {
        ////////////////// STandingPhase //////////////////
        returnXValue = CalculateXValues(length_of_STanding_phase, vel_of_body, t);
        returnZValue = -height;

        } else if (t <= T/4*3) {
        ////////////////// SWingPhase //////////////////
        //// Solve x ////
        // solve undetermined coefficients (double S1[6] = {a1, b1, c1, d1, e1, f1};)
        solve(d1, e1, f1, T, singular1, B_val1, S1);
        returnXValue = CalculateValues(S1, t, T, SW_x_case);

        //// Solve z ////
        // solve undetermined coefficients (double S2[6] = {a2, b2, c2, d2, e2, f2};)
        solve(d2, e2, f2, T, singular2, B_val2, S2);
        returnZValue = CalculateValues(S2, t, T, SW_z_case);

        } else {
        ////////////////// ReversePhase //////////////////
        //// Solve x ////
        solve(d1, e1, f1, T, singular1, B_val1, S1);
        returnXValue = CalculateValues(S1, t, T, Reverse_x_case);

        //// Solve z ////
        solve(d2, e2, f2, T, singular2, B_val2, S2);
        returnZValue = CalculateValues(S2, t, T, Reverse_z_case);
        }
        xVal = returnXValue;
        zVal = returnZValue;
    }

    //////////////////// for Kinematics ////////////////////

    double CalculateKinematics(double xVal, double zVal, int cases)
    {
        double len_hip = 250, len_knee = 250;
        zVal = -zVal;

        // Calculate Knee Joint Value using Inverse Kinematics
        double costh3 = (pow(xVal, 2) + pow(zVal, 2) - pow(len_hip, 2) - pow(len_knee ,2)) / (2*len_hip*len_knee);
        double knee_degree = acos(costh3);

        // Calculate Hip Joint Value using Inverse Kinematics
        double hip_degree = atan2(zVal, xVal) - atan2(len_knee*sin(knee_degree), len_hip + len_knee*cos(knee_degree));

        knee_degree -= M_PI_2; // because of the difference between Kinematics theory and joint of the urdf

        if (cases == 1)
            return hip_degree;
        else
            return knee_degree;
    }

    //////////////////// PID Control Function ////////////////////

    double PID(double kp, double kd, double desiered_pos, int index)
    {
        double output_torque = kp*(desiered_pos - joint_pos[index]) + kd*(0 - joint_vel[index]);
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
        double LF_hip_degree  = CalculateKinematics(xVal, zVal, 1);
        double LF_knee_degree = CalculateKinematics(xVal, zVal, 2);

        double RF_scap_degree = -angle[0];
        double RF_hip_degree  = CalculateKinematics(xVal_counter, zVal_counter, 1);
        double RF_knee_degree = CalculateKinematics(xVal_counter, zVal_counter, 2);

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
    rclcpp::spin(std::make_shared<JointControl>());
    rclcpp::shutdown();
    return 0;
}
