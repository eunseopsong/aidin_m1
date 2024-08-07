#include "convexMPC.h"
#include "func.h"

using namespace std;
using namespace Eigen;
using Eigen::VectorXf;

#define SAMPLING_TIME 0.01
#define DoF 3

/////////////////////// Initialization ////////////////////////
// variable definition
array<double, 6> body_pose{};
array<double, 12> joint_pos{};
array<double, 12> joint_vel{};
array<double, 3> body_pos{};
array<double, 3> body_vel{};
array<double, 9> imu{};
array<double, 4> contact{};
array<double, 3> command{};
array<double, 4> dist{};

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


void SplineTrajectory(double t, double T, double vel_of_body, double &xVal, double &zVal)
{
    /////////////////////// Initializing ////////////////////////

    double length_of_STanding_phase = vel_of_body * T /2;
    double height = 370;
    // double height = 400;

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
    double B_val2[3] = {0, -(4*height/5 +f2), 0};
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

void InverseKinematics3D(double px, double py, double pz, double d1, double l2, double l3, double* th)
{
    pz = -pz;

    // Calculate Scap Joint Value using Inverse Kinematics
    th[0] = fabs( atan2( py, px ) - atan2( d1, fabs(py) ) ) - M_PI_2; // fabs : 절댓값

    // Calculate Knee Joint Value using Inverse Kinematics
    float Ld = sqrt(pow(px, 2) + pow(py, 2) + pow(pz, 2));
    double costh3 = (pow(Ld, 2) - pow(d1, 2) - pow(l2, 2) - pow(l3, 2)) / (2*l2*l3);
    th[2] = acos(costh3);

    // Calculate Hip Joint Value using Inverse Kinematics
    th[1] = ( atan2( pz , sqrt( pow(px, 2) + pow(py, 2) - pow(d1, 2) ) ) - atan2( l3*sin(th[2]), l2 + l3*cos(th[2]) ) );

    th[1] += M_PI_2;
    th[2] -= M_PI_2;
}

///////////////// for Torque Calculation /////////////////

double PDControl(double Kp, double Kd, double target_pos, double current_pos, double current_vel)
{
	double torque_limit = 100;
	double PD_torque = Kp*(target_pos - current_pos) + Kd*(0 - current_vel);

    return min(PD_torque, torque_limit);
}

//////////////// FeedForward Control Function ////////////////

double FFControl(double Kp, double Kd, double th[3], int j_type, int j_idx)
{
    Matrix3d Ic1, Ic2, Ic3, M, C, B;   // 3x3 행렬

    Vector3d PD, joint_square, joint_multiple, G, torque_desired;  // 3x1 벡터

    double m1  = 2.739, m2  = 0.615, m3  = 0.343;
    double L1  = 0.095, L2  = 0.250; // double L3  = 0.250;
    double Lg1 = 0.03106445, Lg2 = 0.06456779, Lg3 = 0.07702597;

    double PD_term_1 = Kp*(th[0] - joint_pos[j_idx]) + Kd*(0 - joint_vel[j_idx]);
    double PD_term_2 = Kp*(th[1] - joint_pos[j_idx+1]) + Kd*(0 - joint_vel[j_idx+1]);
    double PD_term_3 = Kp*(th[2] - joint_pos[j_idx+2]) + Kd*(0 - joint_vel[j_idx+2]);

    Ic1 <<  0.018386717, -0.000009042, -0.000004977,
            -0.000009042,  0.020489644, -0.000009312,
            -0.000004977, -0.000009312,  0.008551259;
    Ic2 <<  0.000769544, -0.000269033, -0.000074609,
           -0.000269033,  0.007987559, -0.000017663,
           -0.000074609, -0.000017663,  0.008526217;
    Ic3 <<  0.000092851,  0.000016513, -0.000000007,
            0.000016513,  0.004330285,  0.000000016,
           -0.000000007,  0.000000016,  0.004412529;

    M << Ic1(0,0)+Ic2(0,0)+Ic3(0,0)+ L1*pow(m2,2) + L1*pow(m3,2) + Lg1*pow(m1,2), L1*Lg3*m3*cos(th[0])*cos(th[1])*sin(th[2]) - Ic3(0,2) - L1*L2*m3*cos(th[0])*cos(th[1]) - L1*Lg2*m2*cos(th[0])*cos(th[1]) - Ic2(0,2) + L1*Lg3*m3*cos(th[0])*cos(th[2])*sin(th[1]), L1*Lg3*m3*cos(th[0])*cos(th[1])*sin(th[2]) - Ic3(0,2) - Ic2(0,2) + L1*Lg3*m3*cos(th[0])*cos(th[2])*sin(th[1]),
         L1*Lg3*m3*cos(th[0])*cos(th[1])*sin(th[2]) - Ic3(2,0) - L1*L2*m3*cos(th[0])*cos(th[1]) - L1*Lg2*m2*cos(th[0])*cos(th[1]) - Ic2(2,0) + L1*Lg3*m3*cos(th[0])*cos(th[2])*sin(th[1]), m3*pow(L2,2) - 2*m3*sin(th[2])*L2*Lg3 + m2*pow(Lg2,2) + m3*pow(Lg3,2) + Ic2(2,2) + Ic3(2,2), m3*pow(Lg3,2) - L2*m3*sin(th[2])*Lg3 + Ic2(2,2) + Ic3(2,2),
         L1*Lg3*m3*cos(th[0])*cos(th[1])*sin(th[2]) - Ic3(2,0) + L1*Lg3*m3*cos(th[0])*cos(th[2])*sin(th[1]), m3*pow(Lg3,2) - L2*m3*sin(th[2])*Lg3 + Ic3(2,2), m3*pow(Lg3,2) + Ic3(2,2);
    PD << PD_term_1, PD_term_2, PD_term_3;

    C << 0, L1*cos(th[0])*(L2*m3*sin(th[1]) + Lg2*m2*sin(th[1]) + Lg3*m3*cos(th[1] + th[2])), L1*Lg3*m3*cos(th[1] + th[2])*cos(th[0]),
         L1*sin(th[0])*(L2*m3*cos(th[1]) - Lg3*m3*sin(th[1] + th[2]) + Lg2*m2*cos(th[1])), 0, -L2*Lg3*m3*cos(th[2]),
        -L1*Lg3*m3*sin(th[1] + th[2])*sin(th[0]), L2*Lg3*m3*cos(th[2]), 0;
    joint_square << pow(joint_vel[j_idx], 2), pow(joint_vel[j_idx+1], 2), pow(joint_vel[j_idx+2], 2);

    B << 0, 0,  2*L1*Lg3*m3*cos(th[1] + th[2])*cos(th[0]),
         0, 0, -2*L2*Lg3*m3*cos(th[2]),
         0, 0,  0;
    joint_multiple << joint_vel[j_idx]*joint_vel[j_idx+1], joint_vel[j_idx]*joint_vel[j_idx+2], joint_vel[j_idx+1]*joint_vel[j_idx+2];

    G << (981*sin(th[0])*(L1*m2 + L1*m3 + Lg1*m1))/100 - (981*cos(th[0])*(L1*m2 + Lg1*m1))/100,
         (981*L2*m3*cos(th[1]))/100 - (981*Lg3*m3*sin(th[1] + th[2]))/50 - (981*L1*m3*cos(th[0]))/100 - (981*Lg2*m2*sin(th[1]))/100 + (981*Lg2*m2*cos(th[1]))/100,
        -(981*m3*(Lg3*cos(th[1] + th[2]) + L2*sin(th[1])))/100 - (981*Lg3*m3*cos(th[1] + th[2]))/100;

    torque_desired = M*PD + C*joint_square + B*joint_multiple + G;

    double torque_limit = 100;
    if (j_type == 0){
        return min(torque_desired[0], torque_limit);
    } else if (j_type == 1) {
        return min(torque_desired[1], torque_limit);
    } else {
        return min(torque_desired[2], torque_limit);
    }
}



double runMPC(double th[3]) {
    // Initialize the MPC controller with a prediction horizon and time step
    convexMPC mpc(10, 0.05);

    // Set the initial state of the robot
    Eigen::VectorXd initialState(13);
    initialState << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    mpc.setInitialState(initialState);

    // Define the reference state and control inputs
    Eigen::VectorXd x_ref(13);
    x_ref << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; // Example reference state
    Eigen::VectorXd u_ref(3 * mpc.getControlInputs().size());
    u_ref << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1; // Example control inputs

    // Update the dynamics matrices based on the current state
    Eigen::MatrixXd A_new = Eigen::MatrixXd::Identity(13, 13); // Example dynamics matrix A
    Eigen::MatrixXd B_new = Eigen::MatrixXd::Identity(13, 3 * mpc.getControlInputs().size() / 3); // Example dynamics matrix B
    mpc.updateDynamicsMatrices(A_new, B_new);

    // Compute the MPC control inputs
    mpc.computeMPC(x_ref, u_ref);

    // Get the control inputs to be applied
    Eigen::VectorXd controlInputs = mpc.getControlInputs();

    // 관절 토크 값만을 반환
    // return controlInputs[0]; // 필요에 따라 적절한 인덱스를 선택
    th[0] = 0; th[1] = 0; th[2] = 0;
    return 0;
}


void CalculateTorqueStanding(double* output_torque, const std::array<double, 3>& Kp, const std::array<double, 3>& Kd, double t)
{
    const std::array<double, 3> target_pos = {0, 0.833070, -0.095344};  // t = T/4 (height=370) success
    const bool is_initial_phase = (t < 2);

    for (int i = 0; i < 12; ++i)
    {
        int joint_type = i % 3;
        int joint_index = (i / 3) * 3;

        if (is_initial_phase && joint_type != 0)
            output_torque[i] = 0;
        else
            output_torque[i] = FFControl(Kp[joint_type], Kd[joint_type], const_cast<double*>(target_pos.data()), joint_type, joint_index);
    }
}

void CalculateTorqueRunning(double* output_torque, const double* target_pos, const std::array<double, 3>& Kp, const std::array<double, 3>& Kd)
{
    // 3개씩 끊어서 저장할 배열
    std::array<std::array<double, 3>, 4> pos;

    // std::copy를 사용하여 배열 복사
    for (int i = 0; i < 4; ++i) {
        std::copy(target_pos + 3 * i, target_pos + 3 * (i + 1), pos[i].begin());
    }

    for (int i = 0; i < 12; ++i)
    {
        int joint_index = i / 3;
        int joint_type = i % 3;

        if (contact[joint_index] == 0)
            output_torque[i] = FFControl(Kp[joint_type], Kd[joint_type], pos[joint_index].data(), joint_type, joint_index * 3);
        else
            output_torque[i] = runMPC(pos[joint_index].data());
    }
}
