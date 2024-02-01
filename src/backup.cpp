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


        // sub_gains = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        //     "/aidin_m1/Gains_sim", 10,
        //     [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        //         kp[0] = msg->data[0];
        //         kp[1] = msg->data[1];
        //         kp[2] = msg->data[2];
        //         kd[0] = msg->data[3];
        //         kd[1] = msg->data[4];
        //         kd[2] = msg->data[5];
        //     });

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
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointpos;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_jointvel;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_simtime;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_torque;
    // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_gains;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_angles;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_desiredpos;

    // Joint angle
    float joint1_pos, joint2_pos, joint3_pos, joint4_pos, joint5_pos, joint6_pos, joint7_pos, joint8_pos, joint9_pos, joint10_pos, joint11_pos, joint12_pos;
    // Joint velocity
    float joint1_vel, joint2_vel, joint3_vel, joint4_vel, joint5_vel, joint6_vel, joint7_vel, joint8_vel, joint9_vel, joint10_vel, joint11_vel, joint12_vel;
    double sim_time;                    // Gazebo simulation time
    double ag[3];                       // Angles


    double q1, q2, q3, l1, l2, l3;      // initial leg angle, leg length

    //////////////////////////////////////////////////////////////////////////////
    ////////////////////// Generate the (Spline) Trajectory //////////////////////
    //////////////////////////////////////////////////////////////////////////////

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
    std::vector<double> CalculateXValues(double v, double tStart, double tEnd, double dt, double l)
    {
        // 계산된 x 좌표를 저장할 배열
        std::vector<double> xValues;

        // 주어진 시간 범위에 따라 x 좌표를 계산하고 배열에 저장
        for (double t = tStart; t <= tEnd; t += dt) {
            double x = (l/2) - v * t;
            xValues.push_back(x);
        }

        return xValues;
    }

    //////////////////// for SWingPhase ////////////////////

    std::vector<double> CalculateValues(double S[], double tStart, double tEnd, double dt, int cases)
    {
        // 계산된 좌표를 저장할 배열
        std::vector<double> SWValues;

        if (cases == 2 || cases == 5) {
            // SWingPhase (x & z)
            for (double t = tStart; t <= tEnd/2; t += dt) {
                double sw = S[0]*pow(t, 5) + S[1]*pow(t, 4) + S[2]*pow(t, 3) + S[3]*pow(t, 2) + S[4]*pow(t, 1) + S[5];
                SWValues.push_back(sw);
            }
        } else {
            // ReversePhase (x & z)
            for (double t = tStart; t <= tEnd/2; t += dt) {
                double sw = S[0]*pow(tEnd/2-t, 5) + S[1]*pow(tEnd/2-t, 4) + S[2]*pow(tEnd/2-t, 3) + S[3]*pow(tEnd/2-t, 2) + S[4]*pow(tEnd/2-t, 1) + S[5];
                SWValues.push_back(sw);
            }
        }

        return SWValues;
    }

    void bezierEndpoint(double t, double &y, double &z)
    {
        /////////////////// Initializing ////////////////////

        double vel_of_body = 1600;
        double T = 0.5;
        double length_of_STanding_phase = vel_of_body * T /2;

        double dt = 0.001;
        double tStart = 0.0;
        double tEnd = T/2;

        double scap_degree, hip_degree, knee_degree;
        double height = 1656/5;
        double scap_length = 80, hip_length = 250, knee_length = 250;
        double InitailxValues = -61.1902;

        int ST_x_case = 1, SW_x_case = 2, Reverse_x_case = 3;
        int ST_z_case = 4, SW_z_case = 5, Reverse_z_case = 6;

        ///////////////////// Solve x /////////////////////
        ////// StandingPhase //////

        // 시간에 따른 x 좌표의 변화 계산
        std::vector<double> STxValues = CalculateXValues(vel_of_body, tStart, T/2, dt, length_of_STanding_phase);

        ////// SwingPhase //////

        // undetermined coefficients (a1*t^5 + b1*t^4 + c1*t^3 + d1*t^2 + e1*t + f1)
        double d1 = 0, e1 = -(vel_of_body), f1 = -length_of_STanding_phase/2;
        double singular1 = T/16;
        double B_val1[3] = {-e1, -(f1 +e1*T/4), 0};
        double S1[6];

        // solve undetermined coefficients (double S1[6] = {a1, b1, c1, d1, e1, f1};)
        solve(d1, e1, f1, T, singular1, B_val1, S1);

        std::vector<double> SWxValues = CalculateValues(S1, tStart, T/2, dt, SW_x_case);

        ////// ReversePhase //////

        // Reverse of SWingPhase
        std::vector<double> REVERSExValues = CalculateValues(S1, tStart, T/2, dt, Reverse_x_case);

        ///////////////////// Solve z /////////////////////
        ////// StandingPhase //////

        // 시간에 따른 z 좌표의 변화 계산 // zValue is constant in STanding Phase.
        std::vector<double> STzValues(T/2/dt, -height);

        ////// SwingPhase //////

        // undetermined coefficients (a2*t^5 + b2*t^4 + c2*t^3 + d2*t^2 + e2*t + f2)
        double d2 = 0, e2 = 0, f2 = -height;
        double singular2 = T/4;
        double B_val2[3] = {0, -(5*height/6 +f2), 0};
        double S2[6];

        // solve undetermined coefficients (double S2[6] = {a2, b2, c2, d2, e2, f2};)
        solve(d2, e2, f2, T, singular2, B_val2, S2);

        std::vector<double> SWzValues = CalculateValues(S2, tStart, T/2, dt, SW_z_case);

        ////// ReversePhase //////

        std::vector<double> REVERSEzValues = CalculateValues(S2, tStart, T/2, dt, Reverse_z_case);


        ////////////////////////////////////////////////////////////////////////////////
        ////////////////////// Bezier Curve Created by jwa & kwon //////////////////////
        ////////////////////////////////////////////////////////////////////////////////

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

        }
        else if (t <= 3*S/2) {

        }
        else {

        }

        y = By;
        z = Bz;
    }


    void CalculateAndPublishTorque()
    {
        // 발 끝 좌표
        double xVal, zVal;
        bezierEndpoint(sim_time, xVal, zVal);
        double yVal = 79;

        // 조인트 각도 계산
        double costh1 = (yVal*l1 + sqrt(yVal*yVal*l1*l1 - (yVal*yVal + zVal*zVal)*(l1*l1 - zVal*zVal))) / (yVal*yVal + zVal*zVal);

        if (costh1 >= -1 && costh1 <= 1)
        {
            double th1 = atan2(sqrt(1 - costh1*costh1), costh1);

            double p_rot_y = xVal*cos(th1) - xVal*sin(th1);
            double p_rot_z = xVal*sin(th1) + xVal*cos(th1);

            double th = atan2(p_rot_z, p_rot_y);
            double l = sqrt(p_rot_y*p_rot_y + p_rot_z*p_rot_z);

            double th2 = M_PI + th - acos((l2*l2 + l*l - l3*l3) / (2*l2*l));
            double th3 = M_PI - acos((l2*l2 + l3*l3 - l*l) / (2*l2*l3));


            std_msgs::msg::Float32MultiArray torque_msg;
            torque_msg.data.clear();

            torque_msg.data.push_back(ag[0]);
            torque_msg.data.push_back(0);
            torque_msg.data.push_back(0);
            torque_msg.data.push_back(-ag[0]);
            torque_msg.data.push_back(0);
            torque_msg.data.push_back(0);
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
            desiredpos_msg.data.push_back(0);
            desiredpos_msg.data.push_back(0);
            desiredpos_msg.data.push_back(ag[0]);
            desiredpos_msg.data.push_back(0);
            desiredpos_msg.data.push_back(0);
            desiredpos_msg.data.push_back(ag[0]);
            desiredpos_msg.data.push_back(0);
            desiredpos_msg.data.push_back(0);

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
