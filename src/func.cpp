#include <iostream>
#include <iomanip>
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

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std;
using namespace Eigen;
using Eigen::VectorXf;

#define SAMPLING_TIME 0.01
#define DoF 3
#define PI 3.14159

/////////////////////// Initializing ////////////////////////
float joint_pos[12];     // Joint Pose
float joint_vel[12];     // Joint Velocity

// double sim_time;      // Gazebo simulation time
double angle[3];         // Angles
double Kp[3], Kd[3];     // Gains

//--------------Command-------------------//
int cmd_mode = 1;
double th_act[DoF] = {0,};
double th_ini[DoF] = {0,};
double th_cmd[DoF] = {0,};
double th_sub[DoF] = {0,};
MatrixXd T03 = MatrixXd::Identity(4, 4);

bool first_callback = true, up = true;

//--------------DH param-------------------//
float L1 = 0.5, L2 = 1, L3 = 1;
float th1_i, th2_i, th3_i,
	  th1, th2, th3,
	  x, y, z;

//--------------PID gain-------------------//
double TargetTor[DoF] = {0, };
double TargetPos[DoF] = {0, };
// double  Kp[3] = {},
//         Ki[3] = {},
//         Kd[3] = {};

//--------------Trajectory Planning-------------------//
bool traj_init = false;
int traj_cnt = 0;
MatrixXf th_out = MatrixXf::Zero(1, DoF); // resize later


//--------------Functions-------------------//
Matrix4d T_craig(float th, float d, float al, float a)
{
	Matrix4d T_craig_;
	T_craig_ << cos(th), 			-sin(th), 			0, 			a,
				sin(th)*cos(al), 	cos(th)*cos(al), 	-sin(al), 	-d*sin(al),
				sin(th)*sin(al), 	cos(th)*sin(al), 	cos(al), 	d*cos(al),
				0, 					0, 					0, 			1;
	return T_craig_;
}

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

    double vel_of_body = 1600;
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

void InverseKinematics3D(float x, float y, float z, bool up_down, double* th_out)
{
	double th1, th2, th3;

	th1 = atan2(y, x);
	if (th1 <= - PI)
		th1 += 2*PI;

	float Ld = sqrt(pow(x,2) + pow(y,2) + pow(z - L1,2));

	if(up_down){
		th3 = acos( (pow(Ld,2) - pow(L2,2) - pow(L3,2)) / (2*L2*L3) );
		th2 = atan2( sqrt(pow(x,2) + pow(y,2)), z - L1 ) - th3 / 2;
	}
	else{
		th3 = - acos( (pow(Ld,2) - pow(L2,2) - pow(L3,2)) / (2*L2*L3) );
		th2 = atan2( sqrt(pow(x,2) + pow(y,2)), z - L1 ) - th3 / 2;
	}

	th_out[0] = th1;
	th_out[1] = th2;
	th_out[2] = th3;
}

double PIDController(double Kp, double Kd, double target_pos, double current_pos, double current_vel)
{
	int torque_limit = 300;
	double PD_torque = Kp*(target_pos - current_pos) + Kd*(0 - current_vel);

	if (PD_torque > torque_limit) {
		PD_torque = torque_limit;
	}
    else if (PD_torque < -torque_limit) {
		PD_torque = -torque_limit;
	}

    return PD_torque;
}

// input DH, output target value
void Forward_K(double* th, MatrixXd& T)
{
	double th1 = th[0];
	double th2 = th[1];
	double th3 = th[2];

	Vector4f theta, d, a, alpha;
	theta << th1, th2 - PI/2, th3, 0;
	d << L1, 0, 0, 0;
	alpha << 0, -PI/2, 0, 0;
	a << 0, 0, L2, L3;

	T = Matrix4d::Identity();

	for (int i = 0; i < DoF + 1; i++)
	{
		T *= T_craig(theta(i), d(i), alpha(i), a(i));
	}
}

double InverseKinematics2D(double xVal, double zVal, int cases)
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

void Traj_joint(double* th_ini, double* th_cmd, MatrixXf& th_out)
{
	// th_out row: 3dof / column: interpolated position

	double vel_des = PI/6; // rad/s
	double max_error = 0;
	for (int i = 0; i < DoF; i++) {
		if(max_error < fabs(th_cmd[i] - th_ini[i]))
			max_error = fabs(th_cmd[i] - th_ini[i]);
	}

	double Tf = max_error / vel_des; // Use maximum error of angle
	double step = round(Tf / SAMPLING_TIME);
	th_out.resize(step, th_out.cols());

	for (int i = 0; i < DoF; i++)
	{
		RowVectorXf inter_pos = RowVectorXf::LinSpaced(step, th_ini[i], th_cmd[i]);
		th_out.block(0, i, step, 1) = inter_pos.transpose();
	}
}

