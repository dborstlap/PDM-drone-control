import numpy as np
import cvxpy as cp
import constants

Q = np.eye(3)
R = np.ones(4) * 0.3


def mpc(quadrotor, x_current, x_target, horizon=50):
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

    x = cp.Variable((12, horizon + 1))
    u = cp.Variable((4, horizon))

    # cost and constraints at each time step
    for n in range(horizon):
        cost += cp.quad_form((x[0:2, n + 1] - x_target), Q)
        cost += cp.quad_form(u[:, n], R)
        constraints += [x[:, n + 1] == quadrotor.A @ x[:, n] + quadrotor.B @ u[:, n]]
        constraints += [u[:, n] >= quadrotor.u_min]
        constraints += [u[:, n] <= quadrotor.u_max]

    # constraints valid for all time steps
    constraints += [x[:, 0] == x_current]

    # run solver
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver=cp.OSQP)

    return u[:, 0].value, x[:, :].value
