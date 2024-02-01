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

    if (cases == 2 || cases == 5)
    {
        // SWingPhase (x & z)
        for (double t = tStart; t <= tEnd/2; t += dt)
        {
            double sw = S[0]*pow(t, 5) + S[1]*pow(t, 4) + S[2]*pow(t, 3) + S[3]*pow(t, 2) + S[4]*pow(t, 1) + S[5];
            SWValues.push_back(sw);
        }
    }
    else
    {
        // ReversePhase (x & z)
        for (double t = tStart; t <= tEnd/2; t += dt)
        {
            double sw = S[0]*pow(tEnd/2-t, 5) + S[1]*pow(tEnd/2-t, 4) + S[2]*pow(tEnd/2-t, 3) + S[3]*pow(tEnd/2-t, 2) + S[4]*pow(tEnd/2-t, 1) + S[5];
            SWValues.push_back(sw);
        }
    }

    return SWValues;
}

///////////////////// for Print /////////////////////

void PrintVector(double tStart, double T, double dt, std::vector<double> values, int cases)
{
    // 결과 출력
    if (cases == 1 || cases == 4)
    {
        if (cases == 1)
            std::cout << "Time\tST-X-Coordinate" << std::endl;
        else
            std::cout << "Time\tST-Z-Coordinate" << std::endl;

        for (double t = tStart; t <= T/2; t += dt)
            std::cout << t << "\t" << values[(t - tStart) / dt] << std::endl;
    }

    else
    {
        if (cases == 2)
            std::cout << "Time\tSW-X-Coordinate" << std::endl;
        else if (cases == 3)
            std::cout << "Time\tReverse-X-Coordinate" << std::endl;
        else if (cases == 5)
            std::cout << "Time\tSW-Z-Coordinate" << std::endl;
        else if (cases == 6)
            std::cout << "Time\tReverse-Z-Coordinate" << std::endl;

        for (double t = tStart; t <= T/4; t += dt)
            std::cout << t << "\t" << values[(t - tStart) / dt] << std::endl;
    }

    // Vector 길이 확인
    std::cout << "Vector의 길이: " << values.size() << std::endl << std::endl;
}

int main()
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

    // 좌표 출력
    PrintVector(tStart, T, dt, STxValues, ST_x_case);

    ////// SwingPhase //////

    // undetermined coefficients (a1*t^5 + b1*t^4 + c1*t^3 + d1*t^2 + e1*t + f1)
    double d1 = 0, e1 = -(vel_of_body), f1 = -length_of_STanding_phase/2;
    double singular1 = T/16;
    double B_val1[3] = {-e1, -(f1 +e1*T/4), 0};
    double S1[6];

    // solve undetermined coefficients (double S1[6] = {a1, b1, c1, d1, e1, f1};)
    solve(d1, e1, f1, T, singular1, B_val1, S1);

    std::vector<double> SWxValues = CalculateValues(S1, tStart, T/2, dt, SW_x_case);
    PrintVector(tStart, T, dt, SWxValues, SW_x_case);

    ////// ReversePhase //////

    // Reverse of SWingPhase
    std::vector<double> REVERSExValues = CalculateValues(S1, tStart, T/2, dt, Reverse_x_case);
    PrintVector(tStart, T, dt, REVERSExValues, Reverse_x_case);

    ///////////////////// Solve z /////////////////////
    ////// StandingPhase //////

    // 시간에 따른 z 좌표의 변화 계산 // zValue is constant in STanding Phase.
    std::vector<double> STzValues(T/2/dt, -height);

    PrintVector(tStart, T, dt, STzValues, ST_z_case);

    ////// SwingPhase //////

    // undetermined coefficients (a2*t^5 + b2*t^4 + c2*t^3 + d2*t^2 + e2*t + f2)
    double d2 = 0, e2 = 0, f2 = -height;
    double singular2 = T/4;
    double B_val2[3] = {0, -(5*height/6 +f2), 0};
    double S2[6];

    // solve undetermined coefficients (double S2[6] = {a2, b2, c2, d2, e2, f2};)
    solve(d2, e2, f2, T, singular2, B_val2, S2);

    std::vector<double> SWzValues = CalculateValues(S2, tStart, T/2, dt, SW_z_case);
    PrintVector(tStart, T, dt, SWzValues, SW_z_case);

    ////// ReversePhase //////

    std::vector<double> REVERSEzValues = CalculateValues(S2, tStart, T/2, dt, Reverse_z_case);
    PrintVector(tStart, T, dt, REVERSEzValues, Reverse_z_case);

    return 0;
}
