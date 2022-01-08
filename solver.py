import numpy as np
import cvxpy as cp

Q = np.eye(3)
R = np.eye(4) * 0.3


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
    obstacle = cp.Variable((6, horizon), boolean=True)
    slack = cp.Variable((6, horizon))
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
        # cost of using slack
        cost += cp.quad_form(slack[:, n], np.eye(6) * slack_penalty)

        # constraints for quadrotor dynamics and actuation limits
        constraints += [x[:, n + 1] == quadrotor.A @ x[:, n] + quadrotor.B @ u[:, n] + quadrotor.G]
        constraints += [u[:, n] >= quadrotor.u_min]
        constraints += [u[:, n] <= quadrotor.u_max]
        constraints += [x[2, n] >= 0]
        constraints += [x[6:9, n] <= np.array([quadrotor.phi_max, quadrotor.theta_max, quadrotor.omega_max])]
        constraints += [x[6:9, n] >= -np.array([quadrotor.phi_max, quadrotor.theta_max, quadrotor.omega_max])]

        # example of how to constrain for obstacle avoidance
        # if the quad is on the right side of the obstacle, then the binary variable will be 0, otherwise 1
        constraints += [1 - x[0, n] + slack[0, n] >= margin * obstacle[0, n]]
        constraints += [x[0, n] - 2 + slack[1, n] >= margin * obstacle[1, n]]
        constraints += [1 - x[1, n] + slack[2, n] >= margin * obstacle[2, n]]
        constraints += [x[1, n] - 2 + slack[3, n] >= margin * obstacle[3, n]]
        constraints += [-1 - x[2, n] + slack[4, n] >= margin * obstacle[4, n]]
        constraints += [x[2, n] - 2.5 + slack[5, n] >= margin * obstacle[5, n]]
        # ensure that at least one binary variable is set to zero for the obstacle, i.e. the obstacle is avoided for at
        # least one plane of the obstacle
        constraints += [(obstacle[0, n] + obstacle[1, n]
                         + obstacle[2, n] + obstacle[3, n]
                         + obstacle[4, n] + obstacle[5, n]
                         ) <= 5]
        constraints += [slack[:, n] >= np.zeros(6)]

    # constraints valid for all time steps
    constraints += [x[:, 0] == x_current]

    # run solver
    problem = cp.Problem(cp.Minimize(cost), constraints)

    problem.solve(solver=cp.CPLEX)

    return u[:, 0].value, x[:, :].value
