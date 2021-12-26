import numpy as np
import cvxpy as cp
import constants

Q = np.eye(3)
R = np.ones(4) * 0.3


def mpc(quadrotor, x_current, x_target):
    cost = 0.
    constraints = []

    x = cp.Variable((12, constants.N + 1))
    u = cp.Variable((4, constants.N))

    for n in range(constants.N):
        cost += cp.quad_form((x[0:2, n + 1] - x_target), Q)
        cost += cp.quad_form(u[:, n], R)
        constraints += [x[:, n + 1] == quadrotor.A @ x[:, n] + quadrotor.B @ u[:, n]]
        constraints += [u[:, n] >= quadrotor.u_min]
        constraints += [u[:, n] <= quadrotor.u_max]

    constraints += [x[:, 0] == x_current]

    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver=cp.OSQP)

    return u[:, 0].value, x[:, 1].value, x[:, :].value
