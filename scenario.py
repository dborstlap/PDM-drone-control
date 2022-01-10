import numpy as np

import constants
from obstacles import Meteorite

filler = Meteorite([-10000, -10000, -10000], [0, 0, 0], 1)


def four_meteorites():
    return [
        Meteorite([4, 6, 2.5], [0.75, -2.5, 0.5], 1),
        Meteorite([1, 6, 1.5], [3.5, 0, 0], 2),
        Meteorite([4, 6, 4.5], [0.25, -0.25, -0.5], 2),
        Meteorite([5, 2, 0.0], [-0.5, 0.5, 0.75], 1.5)
    ]


def default_obstacle_set(n_obstacles, T):
    # return np.array([
    #     np.ones(T) * filler.pos[0] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[0],
    #     np.ones(T) * filler.pos[1] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[1],
    #     np.ones(T) * filler.pos[2] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[2],
    #     np.ones(T) * filler.size + constants.quadrotor_size,
    #     np.ones(T) * filler.pos[0] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[0],
    #     np.ones(T) * filler.pos[1] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[1],
    #     np.ones(T) * filler.pos[2] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[2],
    #     np.ones(T) * filler.size + constants.quadrotor_size,
    #     np.ones(T) * filler.pos[0] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[0],
    #     np.ones(T) * filler.pos[1] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[1],
    #     np.ones(T) * filler.pos[2] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[2],
    #     np.ones(T) * filler.size + constants.quadrotor_size,
    #     np.ones(T) * filler.pos[0] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[0],
    #     np.ones(T) * filler.pos[1] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[1],
    #     np.ones(T) * filler.pos[2] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[2],
    #     np.ones(T) * filler.size + constants.quadrotor_size,
    # ])
    return np.tile(np.array([np.ones(T) * filler.pos[0] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[0],
                             np.ones(T) * filler.pos[1] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[1],
                             np.ones(T) * filler.pos[2] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[2],
                             np.ones(T) * filler.size + constants.quadrotor_size]), (n_obstacles, 1))
