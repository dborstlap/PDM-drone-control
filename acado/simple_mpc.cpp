#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>
#include <cmath>
#include <iostream>

using namespace std;

USING_NAMESPACE_ACADO

int main() {

    // Constants
    const double dt = 0.1;
    const double m = 0.030;
    const double g = 9.81;
    const double L = 0.046;

    const double I_xx = 1.43 * pow(10, -5);
    const double I_yy = 1.43 * pow(10, -5);
    const double I_zz = 2.89 * pow(10, -5);

    const double k_F = 6.11 * pow(10, -8);
    const double k_M = 1.5 * pow(10, -9);
    const double F_max = 2.5 * m * g;
    const double omega_max = sqrt(F_max / (4 * k_F));

    // State and input variables
    DifferentialState x;
    DifferentialState y;
    DifferentialState z;
    DifferentialState x_dot;
    DifferentialState y_dot;
    DifferentialState z_dot;
    DifferentialState phi;
    DifferentialState theta;
    DifferentialState psi;
    DifferentialState phi_dot;
    DifferentialState theta_dot;
    DifferentialState psi_dot;

    Control u1;
    Control u2;
    Control u3;
    Control u4;

    // Online data
    OnlineData od[1][3];

    // Dynamics:
    DifferentialEquation f;
    f << dot(x) == x_dot;
    f << dot(y) == y_dot;
    f << dot(z) == z_dot;
    f << dot(x_dot) == (cos(psi) * sin(theta) + cos(theta) * sin(phi) * sin(psi)) * u1 / m;
    f << dot(y_dot) == (sin(psi) * sin(theta) - cos(theta) * sin(phi) * cos(psi)) * u1 / m;
    f << dot(z_dot) == cos(theta) * cos(phi) * u1 / m - g;
    f << dot(phi) == phi_dot;
    f << dot(theta) == theta_dot;
    f << dot(psi) == psi_dot;
    f << dot(phi_dot) == (-I_yy + I_zz) / I_xx * theta * psi + u2 / I_xx;
    f << dot(theta_dot) == (I_xx - I_zz) / I_yy * phi * psi + u3 / I_yy;
    f << dot(psi_dot) == (-I_xx + I_yy) / I_zz * phi * theta + u4 / I_zz;

    // Actuation limits
    const double u1_min = 0;
    const double u2_min = -L * k_F * pow(omega_max, 2);
    const double u3_min = -L * k_F * pow(omega_max, 2);
    const double u4_min = -2 * k_M * pow(omega_max, 2);
    const double u1_max = F_max;
    const double u2_max = L * k_F * pow(omega_max, 2);
    const double u3_max = L * k_F * pow(omega_max, 2);
    const double u4_max = 2 * k_M * pow(omega_max, 2);

    const double phi_max = asin(m * g);
    const double theta_max = phi_max;
    const double psi_max = 2 / 9 * M_PI;

    //
    // Weighting matrices and reference functions (acadoVariables.y)
    //
    Function rf;
    Function rfN;

    rf << x << y << z << x_dot << y_dot << z_dot << u1 << u2 << u3 << u4;
    rfN << x << y << z << x_dot << y_dot << z_dot;

    // horizon
    const int N = 30;
    // integration steps
    const int Ni = 4;

    // Cost weights
    BMatrix W = eye<bool>(rf.getDim());
    BMatrix WN = eye<bool>(rf.getDim()-4);

    for (int i=6; i < rf.getDim(); i++) {
        W(i, i) = 0.3;
    }

    OCP ocp(0, N * dt, N);

    // dynamics constraint
    ocp.subjectTo(f);

    // actuation constraints
    ocp.subjectTo(u1_min <= u1 <= u1_max);
    ocp.subjectTo(u2_min <= u2 <= u2_max);
    ocp.subjectTo(u3_min <= u3 <= u3_max);
    ocp.subjectTo(u4_min <= u4 <= u4_max);
    ocp.subjectTo(-phi_max <= phi <= phi_max);
    ocp.subjectTo(-theta_max <= theta <= theta_max);
    ocp.subjectTo(-psi_max <= psi <= psi_max);

    std::cout << od << std::endl;

    // obstacle constraints
//     for(int i = 0; i < 1; i++){
//       ocp.subjectTo(sqrt(
//           (x - od[i][0]) * (x - od[i][0]) +
//           (y - od[i][1]) * (y - od[i][1]) +
//           (z - od[i][2]) * (z - od[i][2]))
//           >= 0.75
//       );
//     }

     ocp.subjectTo(sqrt((x - 1) * (x - 1) + (y - 1) * (y - 1) + (z - 1) * (z - 1)) >= 0.5);
     ocp.subjectTo(sqrt((x - 2) * (x - 2) + (y - 3) * (y - 3) + (z - 1) * (z - 1)) >= 0.5);
     ocp.subjectTo(sqrt((x - 0) * (x - 0) + (y - 3) * (y - 3) + (z - 2) * (z - 2)) >= 0.5);

    // minimize for cost
    ocp.minimizeLSQ(W, rf);
    ocp.minimizeLSQEndTerm(WN, rfN);

    // Export the code:
    ocp.setNOD(1);
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
    mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);
    mpc.set(HOTSTART_QP, YES);
    mpc.set(GENERATE_TEST_FILE, YES);
    mpc.set(GENERATE_MAKE_FILE, YES);
    mpc.set(GENERATE_MATLAB_INTERFACE, YES);
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
    mpc.set(FIX_INITIAL_STATE, YES);

    if (mpc.exportCode("simple_mpc_export") != SUCCESSFUL_RETURN)
        exit(EXIT_FAILURE);

    mpc.printDimensionsQP();

    return EXIT_SUCCESS;
}



