#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

// Eigen 라이브러리를 사용하기 위해 행렬 및 벡터 타입 정의
using namespace Eigen;

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
    double returnDegree;
    double len_hip = 250, len_knee = 250;

    zVal = -zVal;

    // Calculate Knee Joint Value using Inverse Kinematics
    double costh3 = (pow(xVal, 2) + pow(zVal, 2) - pow(len_hip, 2) - pow(len_knee ,2)) / (2*len_hip*len_knee);
    double knee_degree = acos(costh3);

    // Calculate Hip Joint Value using Inverse Kinematics
    double hip_degree = atan2(zVal, xVal) - atan2(len_knee*sin(knee_degree), len_hip + len_knee*cos(knee_degree));

    knee_degree -= M_PI_2;

    if (cases == 1)
        returnDegree = hip_degree;
    else
        returnDegree = knee_degree;

    return returnDegree;
}

///////////////////////////////////////////////////////
////////////////////// for Print //////////////////////
///////////////////////////////////////////////////////

void PrintVector(std::vector<double> values, int cases)
{
    // 결과 출력
    if (cases == 1 || cases == 4) {
        std::cout << "STandingPhase-Values" << std::endl;

        for (const auto& element : values)
            std::cout << element << std::endl;
    } else {
        if (cases == 2 || cases == 5)
            std::cout << "SWingPhase-Values" << std::endl;
        else if (cases == 3 || cases == 6)
            std::cout << "ReversePhase-Values" << std::endl;
        else
            std::cout << "Check Values" << std::endl;

        for (const auto& element : values)
            std::cout << element << std::endl;
    }

    // Vector 길이 확인
    std::cout << "Vector의 길이: " << values.size() << std::endl << std::endl;
}

int main(){
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
    // 좌표값을 저장할 배열 생성
    std::vector<double> STxValues;

    // 주어진 시간 범위에 따라 x 좌표를 계산하고 배열에 저장
    for (double t = 0.0; t <= T/2; t += dt) {
        double x = CalculateXValues(length_of_STanding_phase, vel_of_body, t);
        STxValues.push_back(x);
    }

    ////// SwingPhase //////
    // undetermined coefficients (a1*t^5 + b1*t^4 + c1*t^3 + d1*t^2 + e1*t + f1)
    double d1 = 0, e1 = -(vel_of_body), f1 = -length_of_STanding_phase/2;
    double singular1 = T/16;
    double B_val1[3] = {-e1, -(f1 +e1*T/4), 0};
    double S1[6];

    // solve undetermined coefficients (double S1[6] = {a1, b1, c1, d1, e1, f1};)
    solve(d1, e1, f1, T, singular1, B_val1, S1);

    std::vector<double> SWxValues;

    for (double t = T/2; t <= T/4*3; t += dt) {
        double x = CalculateValues(S1, t, T, SW_x_case);
        SWxValues.push_back(x);
    }

    ////// ReversePhase //////
    // Reverse of SWingPhase
    std::vector<double> REVERSExValues;

    for (double t = T/4*3; t <= T; t += dt) {
        double x = CalculateValues(S1, t, T, Reverse_x_case);
        REVERSExValues.push_back(x);
    }

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

    std::vector<double> SWzValues;

    for (double t = T/2; t <= T/4*3; t += dt) {
        double z = CalculateValues(S2, t, T, SW_z_case);
        SWzValues.push_back(z);
    }

    ////// ReversePhase //////

    std::vector<double> REVERSEzValues;

    for (double t = T/4*3; t <= T; t += dt) {
        double z = CalculateValues(S2, t, T, Reverse_z_case);
        REVERSEzValues.push_back(z);
    }

    ///////////////////// Concatenation /////////////////////
    // 두 벡터를 합칠 벡터 생성
    std::vector<double> xValues;
    std::vector<double> zValues;

    xValues.insert(xValues.end(), STxValues.begin(), STxValues.end());
    xValues.insert(xValues.end(), SWxValues.begin(), SWxValues.end());
    xValues.insert(xValues.end(), REVERSExValues.begin(), REVERSExValues.end());

    zValues.insert(zValues.end(), STzValues.begin(), STzValues.end());
    zValues.insert(zValues.end(), SWzValues.begin(), SWzValues.end());
    zValues.insert(zValues.end(), REVERSEzValues.begin(), REVERSEzValues.end());

    ///// Print Check /////
    // PrintVector(xValues, 0);
    // PrintVector(zValues, 0);

    // PrintVector(STxValues, 1);
    // PrintVector(SWxValues, 2);
    // PrintVector(REVERSExValues, 3);

    // PrintVector(STzValues, 4);
    // PrintVector(SWzValues, 5);
    // PrintVector(REVERSEzValues, 6);

    ////////////////////////////////////////////////////////////////
    ///////////////////// Calculate Kinematics /////////////////////
    ////////////////////////////////////////////////////////////////
    ////// StandingPhase //////
    std::vector<double> SThipDegree;

    for (int i = 0; i < STxValues.size(); i ++) {
        double hip_degree = CalculateKinematics(STxValues[i], STzValues[i], 1);
        SThipDegree.push_back(hip_degree);
    }

    std::vector<double> STkneeDegree;

    for (int i = 0; i < STxValues.size(); i ++) {
        double knee_degree = CalculateKinematics(STxValues[i], STzValues[i], 2);
        STkneeDegree.push_back(knee_degree);
    }

    ////// SwingPhase //////
    std::vector<double> SWhipDegree;

    for (int i = 0; i < SWxValues.size(); i ++) {
        double hip_degree = CalculateKinematics(SWxValues[i], SWzValues[i], 1);
        SWhipDegree.push_back(hip_degree);
    }

    std::vector<double> SWkneeDegree;

    for (int i = 0; i < SWxValues.size(); i ++) {
        double knee_degree = CalculateKinematics(SWxValues[i], SWzValues[i], 2);
        SWkneeDegree.push_back(knee_degree);
    }

    ////// ReversePhase //////
    std::vector<double> REVERSEhipDegree;

    for (int i = 0; i < REVERSExValues.size(); i ++) {
        double hip_degree = CalculateKinematics(REVERSExValues[i], REVERSEzValues[i], 1);
        REVERSEhipDegree.push_back(hip_degree);
    }

    std::vector<double> REVERSEkneeDegree;

    for (int i = 0; i < REVERSExValues.size(); i ++) {
        double knee_degree = CalculateKinematics(REVERSExValues[i], REVERSEzValues[i], 2);
        REVERSEkneeDegree.push_back(knee_degree);
    }

    ///// Print Check Step by Step /////
    // PrintVector(SThipDegree, 1);
    // PrintVector(SWhipDegree, 2);
    // PrintVector(REVERSEhipDegree, 3);

    PrintVector(STkneeDegree, 4);
    PrintVector(SWkneeDegree, 5);
    PrintVector(REVERSEkneeDegree, 6);

    ///// Concatenation /////
    std::vector<double> HipDegree;
    std::vector<double> KneeDegree;

    ///// Print Check ALL /////
    // std::cout << "HipDegree Values: " << std::endl;
    // PrintVector(HipDegree, 0);

    // std::cout << "KneeDegree zValues: " << std::endl;
    // PrintVector(KneeDegree, 0);

    return 0;
}
