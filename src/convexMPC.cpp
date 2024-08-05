#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <qpOASES.hpp>

using namespace Eigen;
using namespace qpOASES;

class convexMPC {
public:
    convexMPC(int horizonLength, double dt)
        : horizonLength(horizonLength), dt(dt) {
        // Initialize the state and control vectors
        x.resize(13); // State vector (position, orientation, velocities)
        u.resize(3 * numLegs); // Control vector (forces)
        A.resize(13, 13); // State transition matrix
        B.resize(13, 3 * numLegs); // Control input matrix
        Q = MatrixXd::Identity(13, 13); // State weighting matrix
        R = MatrixXd::Identity(3 * numLegs, 3 * numLegs); // Control weighting matrix
    }

    void setInitialState(const VectorXd &initialState) {
        x = initialState;
    }

    void updateDynamicsMatrices(const MatrixXd &A_new, const MatrixXd &B_new) {
        A = A_new;
        B = B_new;
    }

    void computeMPC(const VectorXd &x_ref, const VectorXd & /* u_ref */) {
        // Define the quadratic programming problem here
        // H is the Hessian matrix, g is the gradient vector
        MatrixXd H = 2 * (B.transpose() * Q * B + R);
        VectorXd g = 2 * B.transpose() * Q * (A * x - x_ref);

        // Constraints matrices C and D
        MatrixXd C = MatrixXd::Identity(3 * numLegs * horizonLength, 3 * numLegs * horizonLength); // Fill in constraints based on problem specifics
        VectorXd lb = VectorXd::Constant(3 * numLegs * horizonLength, -1e2); // Lower bounds for control inputs
        VectorXd ub = VectorXd::Constant(3 * numLegs * horizonLength, 1e2); // Upper bounds for control inputs

        // Set up the QP problem
        QProblem qp(3 * numLegs * horizonLength, C.rows());
        Options options;
        qp.setOptions(options);
        int nWSR = 100;
        real_t *H_real = new real_t[H.size()];
        real_t *g_real = new real_t[g.size()];
        real_t *C_real = new real_t[C.size()];
        real_t *lb_real = new real_t[lb.size()];
        real_t *ub_real = new real_t[ub.size()];

        // Copy data to real_t arrays
        std::copy(H.data(), H.data() + H.size(), H_real);
        std::copy(g.data(), g.data() + g.size(), g_real);
        std::copy(C.data(), C.data() + C.size(), C_real);
        std::copy(lb.data(), lb.data() + lb.size(), lb_real);
        std::copy(ub.data(), ub.data() + ub.size(), ub_real);

        // Initialize the QP problem
        qp.init(H_real, g_real, C_real, lb_real, ub_real, nullptr, nullptr, nWSR);

        // Get the solution
        qp.getPrimalSolution(u.data());

        // Cleanup
        delete[] H_real;
        delete[] g_real;
        delete[] C_real;
        delete[] lb_real;
        delete[] ub_real;
    }

    VectorXd getControlInputs() const {
        return u;
    }

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
};
