import numpy as np
import cvxpy as cp

Q = np.eye(3)
R = np.eye(4) * 10


def mpc(quadrotor, x_current, x_target, obstacle_list, meteorites_list, horizon=50):
    """
    Run the MPC solver for the given quadrotor from current location. The solver will try to close the distance to
    the target location.
    :param meteorites_list: moving obstacles
    :param obstacle_list: static obstacles
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

    if len(obstacle_list) > 0:
        obstacle_binary = cp.Variable((6 * len(obstacle_list), horizon), boolean=True)
        cuboid_slack = cp.Variable((6 * len(obstacle_list), horizon))

    if len(meteorites_list) > 0:
        meteorite_binary = cp.Variable((6 * len(meteorites_list), horizon), boolean=True)
        meteorite_slack = cp.Variable((6 * len(meteorites_list), horizon))

    slack_penalty = 100000
    margin = -1000

    # cost and constraints at each time step
    for n in range(horizon):
        # cost of position
        cost += cp.quad_form((x[:3, n + 1] - x_target[:3]), Q)
        # cost of velocity
        cost += cp.quad_form((x[6:9, n + 1] - x_target[6:9]), Q)
        # cost of inputs
        cost += cp.quad_form(u[:, n], R)
        if len(obstacle_list) > 0:
            cost += cp.quad_form(cuboid_slack[:, n], np.eye(6 * len(obstacle_list)) * slack_penalty)
        if len(meteorites_list) > 0:
            cost += cp.quad_form(meteorite_slack[:, n], np.eye(6 * len(meteorites_list)) * slack_penalty)

        constraints += [x[:, n + 1] == quadrotor.A @ x[:, n] + quadrotor.B @ u[:, n] + quadrotor.G]
        constraints += [u[:, n] >= quadrotor.u_min]
        constraints += [u[:, n] <= quadrotor.u_max]
        constraints += [x[2, n] >= 0]
        constraints += [x[6:9, n] <= np.array([quadrotor.phi_max, quadrotor.theta_max, quadrotor.omega_max])]
        constraints += [x[6:9, n] >= -np.array([quadrotor.phi_max, quadrotor.theta_max, quadrotor.omega_max])]

        for i in range(len(obstacle_list)):
            j = i * 6
            obstacle_list[i].add_constraints(x, n, constraints, margin, obstacle_binary, cuboid_slack, j)

        for i in range(len(meteorites_list)):
            j = i * 6
            meteorites_list[i].add_constraints(x, n, constraints, margin, meteorite_binary, meteorite_slack, j)

        if len(obstacle_list) > 0:
            constraints += [cuboid_slack[:, n] >= np.zeros(6*len(obstacle_list))]
        if len(meteorites_list) > 0:
            constraints += [meteorite_slack[:, n] >= np.zeros(6)]

    # constraints valid for all time steps
    constraints += [x[:, 0] == x_current]

    # run solver
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver=cp.CPLEX)

    return u[:, 0].value, x[:, :].value
