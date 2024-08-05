#ifndef FUNC_H
#define FUNC_H

#include <array>
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

// Function prototypes
void solve(double d, double e, double f, double T, double singularity, double B_val[], double arr[6]);
double CalculateXValues(double l, double v, double t);
double CalculateValues(double S[], double t, double T, int cases);
void SplineTrajectory(double t, double T, double vel_of_body, double &xVal, double &zVal);
double InverseKinematics2D(double xVal, double zVal, int cases);
void InverseKinematics3D(double px, double py, double pz, double d1, double l2, double l3, double* th);
double PDControl(double Kp, double Kd, double target_pos, double current_pos, double current_vel);
double FFControl(double Kp, double Kd, double th[3], int case_, int cri);
double MPC(double th[3]);
void CalculateTorqueStanding(double* output_torque, array<double, 3> Kp, array<double ,3> Kd);
void CalculateTorqueRunning(double* output_torque, double* target_pos, array<double, 3> Kp, array<double ,3> Kd);

#endif // FUNC_H
