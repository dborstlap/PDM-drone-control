import numpy as np
import cvxpy as cp

Q = np.eye(3)
R = np.eye(4) * 0.3


def mpc(quadrotor, x_current, x_target, horizon=20):
    """
    Run the MPC solver for the given quadrotor from current location. The solver will try to close the distance to
    the target location.
    :param quadrotor: the Drone instance
    :param x_current: the current position, list or np.array
    :param x_target: the target position, list or np.array
    :param horizon: number of steps until horizon (default = 50)
    :return: inputs at step 0, predicted trajectory until horizon
    """
    cost = 0.
    constraints = []

    # print(cp.installed_solvers())

    x = cp.Variable((12, horizon + 1))
    u = cp.Variable((4, horizon))
    obstacle = cp.Variable((6, horizon), boolean=True)

    # cost and constraints at each time step
    for n in range(horizon):
        # cost of position
        cost += cp.quad_form((x[:3, n + 1] - x_target[:3]), Q)
        # cost of velocity
        cost += cp.quad_form((x[6:9, n + 1] - x_target[6:9]), Q)
        cost += cp.quad_form(u[:, n], R)
        constraints += [x[:, n + 1] == quadrotor.A @ x[:, n] + quadrotor.B @ u[:, n] + quadrotor.G]
        constraints += [u[:, n] >= quadrotor.u_min]
        constraints += [u[:, n] <= quadrotor.u_max]
        constraints += [x[2, n] >= 0]
        constraints += [x[6:9, n] <= np.array([quadrotor.phi_max, quadrotor.theta_max, quadrotor.omega_max])]
        constraints += [x[6:9, n] >= -np.array([quadrotor.phi_max, quadrotor.theta_max, quadrotor.omega_max])]

        constraints += [1 - x[0, n] >= -10000 * obstacle[0, n]]
        constraints += [x[0, n] - 2 >= -10000 * obstacle[1, n]]
        constraints += [1 - x[1, n] >= -10000 * obstacle[2, n]]
        constraints += [x[1, n] - 2 >= -10000 * obstacle[3, n]]
        constraints += [0.25 - x[2, n] >= -10000 * obstacle[4, n]]
        constraints += [x[2, n] - 2 >= -10000 * obstacle[5, n]]
        constraints += [(obstacle[0, n] + obstacle[1, n]
                         + obstacle[2, n] + obstacle[3, n]
                         + obstacle[4, n] + obstacle[5, n]
                         ) <= 5]

    # constraints valid for all time steps
    constraints += [x[:, 0] == x_current]

    # run solver
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver=cp.CPLEX, verbose=False)

    return u[:, 0].value, x[:, :].value
