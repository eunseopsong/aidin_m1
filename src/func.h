#ifndef FUNC_H
#define FUNC_H

#include <iostream>
#include <iomanip>
#include <chrono>
#include <functional>
#include <memory>

#include <string>
#include <algorithm> // std::copy
#include <array>

#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std;
using namespace Eigen;

// variable declaration
extern array<double, 6> body_pose;
extern array<double, 12> joint_pos;
extern array<double, 12> joint_vel;
extern array<double, 3> body_pos;
extern array<double, 3> body_vel;
extern array<double, 9> imu;
extern array<double, 4> contact;
extern array<double, 3> command;
extern array<double, 4> dist;

// Function set for FootstepTrajectroy generation
void solve(double d, double e, double f, double T, double singularity, double B_val[], double arr[6]);
double CalculateXValues(double l, double v, double t);
double CalculateValues(double S[], double t, double T, int cases);
void SplineTrajectory(double t, double T, double vel_of_body, double &xVal, double &zVal);

// Function set for calculating InverseKineamatics
double InverseKinematics2D(double xVal, double zVal, int cases);
void InverseKinematics3D(double px, double py, double pz, double d1, double l2, double l3, double* th);

// Function set for calculating torque of each joint
double PDControl(double Kp, double Kd, double target_pos, double current_pos, double current_vel);
double FFControl(double Kp, double Kd, double th[3], int case_, int cri);
double runMPC(double th[3]);

// Function set for delivering the torque to control node
void CalculateTorqueStanding(double* output_torque, array<double, 3> Kp, array<double ,3> Kd, double t);
void CalculateTorqueRunning(double* output_torque, double* target_pos, array<double, 3> Kp, array<double ,3> Kd);

#endif // FUNC_H
