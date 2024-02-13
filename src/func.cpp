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

    double InverseKinematics(double xVal, double zVal, int cases)
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

