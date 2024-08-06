#ifndef CONVEXMPC_H
#define CONVEXMPC_H

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <qpOASES.hpp>

using namespace Eigen;
using namespace qpOASES;

class convexMPC {
private:
    int horizonLength;
    double dt;
    int numLegs = 4;
    VectorXd x; // State vector
    VectorXd u; // Control vector
    MatrixXd A; // State transition matrix
    MatrixXd B; // Control input matrix
    MatrixXd Q; // State weighting matrix
    MatrixXd R; // Control weighting matrix

public:
    convexMPC(int horizonLength, double dt);
    void setInitialState(const VectorXd &initialState);
    void updateDynamicsMatrices(const MatrixXd &A_new, const MatrixXd &B_new);
    void computeMPC(const VectorXd &x_ref, const VectorXd &u_ref);
    VectorXd getControlInputs() const;
};

#endif // CONVEXMPC_H
