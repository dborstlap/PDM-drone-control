import numpy as np

import constants
from obstacles import Meteorite

filler = Meteorite([-10000, -10000, -10000], [0, 0, 0], 1)


def default_scenario():
    return [
        Meteorite([4, 6, 2.5], [0.75, -2.5, 0.5], 1),
        Meteorite([1, 6, 1.5], [3.5, 0, 0], 2),
        Meteorite([4, 6, 4.5], [0.25, -0.25, -0.5], 2),
        Meteorite([5, 2, 0.0], [-0.5, 0.5, 0.75], 1.5)
    ]


def dense_scenario():
    return [
        Meteorite([4, 6, 2.5], [0.75, -2.5, 0.5], 1),
        Meteorite([1, 6, 1.5], [2.5, -1, 0], 2),
        Meteorite([4, 6, 2.5], [0.25, -0.75, -0.25], 2),
        Meteorite([5, 2, 0.0], [-0.5, 0.5, 0.75], 1.5),
        Meteorite([3, 0, 2.5], [0, 0.5, -0.25], 1),
        Meteorite([1, 4, 0.5], [0.5, -0.25, 0.25], 1.5)
    ]


def dense_fast():
    return [
        Meteorite([1, 3, 0.5], [0, 0, 0], 1.5),
        Meteorite([3, 4.5, 1.0], [0, 0, 0], 1.5),
        Meteorite([3, 3, 1.0], [0, -1.5, 0], 1.5),
        Meteorite([1, 3, 1.5], [-0.25, -1.0, -0.25], 1.5),
        Meteorite([3, 3, 2.0], [-0.25, -1.5, -0.50], 1.5),
        Meteorite([3, 3, 3.0], [1, -0.5, -0.25], 1.5)
    ]


def dense_extra_fast():
    return [
        Meteorite([1, 3, 0.5], [0, 0, 0], 1.5),
        Meteorite([3, 4.5, 1.0], [0, 0, 0], 1.5),
        Meteorite([3, 3, 1.0], [0, -3, 0], 1.5),
        Meteorite([1, 3, 1.5], [-0.25, -2.0, -0.25], 1.5),
        Meteorite([3, 3, 2.0], [-0.25, -3.0, -0.50], 1.5),
        Meteorite([3, 3, 2.0], [0.5, -1.0, -0.25], 1.5)
    ]


def dense_extra_fast_small_far():
    return [
        Meteorite([1, 3, 0.5], [0, -0.125, 0], 1.0),
        Meteorite([3, 4.5, 1.0], [0, 0, 0], 1.0),
        Meteorite([3, 6, 2.5], [-1, -3, 0], 1.0),
        Meteorite([1, 6, 2.5], [-0.25, -2.0, -0.50], 1.0),
        Meteorite([5, 6, 2.5], [-0.25, -3.0, -0.50], 1.0),
        Meteorite([4, 6, 2.5], [0.5, -1.0, -0.55], 1.0)
    ]


def dense_extra_fast_small():
    return [
        Meteorite([1, 3, 0.5], [0, -0.125, 0], 0.5),
        Meteorite([3, 4.5, 1.0], [0, 0, 0], 0.5),
        Meteorite([3, 3, 1.0], [-1, -3, 0], 0.5),
        Meteorite([1, 3, 1.5], [-0.25, -2.0, -0.50], 0.5),
        Meteorite([3, 3, 2.0], [-0.25, -3.0, -0.50], 0.5),
        Meteorite([3, 3, 2.0], [0.5, -1.0, -0.55], 0.5)
    ]


def dense_extra_fast_medium():
    return [
        Meteorite([1, 3, 0.5], [0, -0.125, 0], 1.5),
        Meteorite([3, 4.5, 1.0], [0, 0, 0], 1.5),
        Meteorite([3, 3, 1.0], [-1, -3, 0], 1.25),
        Meteorite([1, 3, 1.5], [-0.25, -2.0, -0.50], 1.25),
        Meteorite([3, 3, 2.0], [-0.25, -3.0, -0.50], 1.25),
        Meteorite([3, 3, 2.0], [0.5, -1.0, -0.55], 1.25)
    ]


def dense_unstable():
    return [
        Meteorite([3, 3, 0.5], [1, 0, 0.5], 1.5),
        Meteorite([3, 3, 1.0], [1, 1, 0.5], 1.5),
        Meteorite([3, 3, 1.5], [0, -0.75, 0], 1.5),
        Meteorite([3, 3, 2.0], [0, 0.5, -0.25], 1.5),
        Meteorite([3, 3, 2.5], [-0.25, 0.5, -0.25], 1.5),
        Meteorite([3, 3, 3.0], [0.5, -0.25, -0.25], 1.5)
    ]


def default_obstacle_set(n_obstacles, T):
    return np.tile(np.array(
        [np.ones(T) * filler.pos[0] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[0],
         np.ones(T) * filler.pos[1] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[1],
         np.ones(T) * filler.pos[2] + np.arange(0, T * constants.dt_solver, constants.dt_solver) * filler.vel[2],
         np.ones(T) * filler.size + constants.quadrotor_size]), (n_obstacles, 1))
