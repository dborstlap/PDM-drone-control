#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>
#include <cmath>
#include <algorithm>
#include <numeric>

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
    DifferentialState dummy;

    Control u1;
    Control u2;
    Control u3;
    Control u4;

    Control slack_1;
    Control slack_2;
    Control slack_3;
    Control slack_4;

    // Online data
    int N_ONLINE = 16;
    OnlineData obs_1_x;
    OnlineData obs_1_y;
    OnlineData obs_1_z;
    OnlineData obs_1_d;
    OnlineData obs_2_x;
    OnlineData obs_2_y;
    OnlineData obs_2_z;
    OnlineData obs_2_d;
    OnlineData obs_3_x;
    OnlineData obs_3_y;
    OnlineData obs_3_z;
    OnlineData obs_3_d;
    OnlineData obs_4_x;
    OnlineData obs_4_y;
    OnlineData obs_4_z;
    OnlineData obs_4_d;

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
    f << dot(dummy) == slack_1 + slack_2 + slack_3 + slack_4;

    // Actuation limits
    const double u1_min = 0;
    const double u2_min = -L * k_F * pow(omega_max, 2);
    const double u3_min = -L * k_F * pow(omega_max, 2);
    const double u4_min = -2 * k_M * pow(omega_max, 2);
    const double u1_max = F_max;
    const double u2_max = L * k_F * pow(omega_max, 2);
    const double u3_max = L * k_F * pow(omega_max, 2);
    const double u4_max = 2 * k_M * pow(omega_max, 2);

//    const double phi_max = asin(1/2.5);
    const double phi_max = 0.40;
    const double theta_max = phi_max;
    const double psi_max = 2 / 9 * M_PI;

    //
    // Weighting matrices and reference functions (acadoVariables.y)
    //
    Function rf;
    Function rfN;

    rf << x << y << z << x_dot << y_dot << z_dot << u1 << u2 << u3 << u4 << slack_1 << slack_2 << slack_3 << slack_4;
//    rf << x << y << z << x_dot << y_dot << z_dot << u1 << u2 << u3 << u4;
    rfN << x << y << z << x_dot << y_dot << z_dot;

    // horizon
    const int N = 40;
    // integration steps
    const int Ni = 4;

    // Cost weights
    BMatrix W = eye<bool>(rf.getDim());
    BMatrix WN = eye<bool>(rfN.getDim());

//    for (int i=6; i < 12; i++) {
//        W(i, i) = 0.3;
//    }

    OCP ocp(0, N * dt, N);

    // dynamics constraint
    ocp.subjectTo(f);

    // actuation constraints
    ocp.subjectTo(u1_min <= u1 <= u1_max);
    ocp.subjectTo(u2_min <= u2 <= u2_max);
    ocp.subjectTo(u3_min <= u3 <= u3_max);
    ocp.subjectTo(u4_min <= u4 <= u4_max);
    ocp.subjectTo(z >= 0);
    ocp.subjectTo(slack_1 >= 0);
    ocp.subjectTo(-phi_max <= phi <= phi_max);
    ocp.subjectTo(-theta_max <= theta <= theta_max);
    ocp.subjectTo(-psi_max <= psi <= psi_max);

    // obstacle constraints
//    ocp.subjectTo(sqrt((x - obs_1_x) * (x - obs_1_x) + (y - obs_1_y) * (y - obs_1_y) + (z - obs_1_z) * (z - obs_1_z)) - obs_1_d >= 0);
    ocp.subjectTo(sqrt((x - obs_1_x) * (x - obs_1_x) + (y - obs_1_y) * (y - obs_1_y) + (z - obs_1_z) * (z - obs_1_z)) + slack_1 - obs_1_d >= 0);
    ocp.subjectTo(sqrt((x - obs_2_x) * (x - obs_2_x) + (y - obs_2_y) * (y - obs_2_y) + (z - obs_2_z) * (z - obs_2_z)) + slack_2 - obs_2_d >= 0);
    ocp.subjectTo(sqrt((x - obs_3_x) * (x - obs_3_x) + (y - obs_3_y) * (y - obs_3_y) + (z - obs_3_z) * (z - obs_3_z)) + slack_3 - obs_3_d >= 0);
    ocp.subjectTo(sqrt((x - obs_4_x) * (x - obs_4_x) + (y - obs_4_y) * (y - obs_4_y) + (z - obs_4_z) * (z - obs_4_z)) + slack_4 - obs_4_d >= 0);

    // minimize for cost
    ocp.minimizeLSQ(W, rf);
    ocp.minimizeLSQEndTerm(WN, rfN);

    // Export the code:
    ocp.setNOD(N_ONLINE);
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set(INTEGRATOR_TYPE, INT_RK45);
    mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);
    mpc.set(HOTSTART_QP, YES);
//    mpc.set(GENERATE_TEST_FILE, YES);
//    mpc.set(GENERATE_MAKE_FILE, YES);
//    mpc.set(GENERATE_MATLAB_INTERFACE, YES);
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);
    mpc.set(FIX_INITIAL_STATE, YES);

    if (mpc.exportCode("simple_mpc_export") != SUCCESSFUL_RETURN)
        exit(EXIT_FAILURE);

    mpc.printDimensionsQP();

    return EXIT_SUCCESS;
}



