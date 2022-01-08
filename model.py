""" 
name:     model.py
authors:  Dries, Wesley, Tanya, Koen
function: Contains the drone model
"""

# ---------------------------- IMPORTS ---------------------------------
import math
import time
from math import sin, cos
import numpy as np
import random as rd
import pygame as pg
import acado
import constants
import solver
from functions import rotation_matrix, projection, depth_scale, pressed_keys, colors


# --------------------------- DRONE CLASS ---------------------------------
class Drone:
    m = 0.030
    arm_length = 0.046
    k_F = 6.11 * 10 ** -8
    k_M = 1.5 * 10 ** -9
    # J = np.array([
    #     [1.1, 0, 0],
    #     [0, 1, 0],
    #     [0, 0, 1.7]
    # ]) * 10 ** (-3)
    J = np.diag([1.43, 1.43, 2.89]) * 10 ** -5
    F_max = 2.5 * m * constants.g

    phi_max = math.asin(m * constants.g)
    theta_max = phi_max
    psi_max = 2 / 9 * math.pi
    omega_max = math.sqrt(F_max / (4 * k_F))
    # omega_max = 2500

    A = np.eye(12) + np.array([
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, constants.g, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, -constants.g, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]) * constants.dt

    B = np.array([
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [1 / m, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 1 / J[0, 0], 0, 0],
        [0, 0, 1 / J[1, 1], 0],
        [0, 0, 0, 1 / J[2, 2]]
    ]) * constants.dt

    C = np.eye(6)

    D = np.zeros((6, 4))

    G = np.array([0, 0, 0, 0, 0, -constants.g, 0, 0, 0, 0, 0, 0]) * constants.dt

    u_min = np.array([
        0,
        -arm_length * k_F * omega_max ** 2,
        -arm_length * k_F * omega_max ** 2,
        -2 * k_M * omega_max ** 2
    ])
    u_max = np.array([
        F_max,
        arm_length * k_F * omega_max ** 2,
        arm_length * k_F * omega_max ** 2,
        2 * k_M * omega_max ** 2
    ])

    state = np.zeros(12)

    # Initialize some drone parameters
    def __init__(self, X):
        self.X = np.array(X)
        self.pos = np.array([X[0], X[1], X[2]])
        self.drone_rotation_matrix = np.eye(3)

    def update_state(self, inputs, model='non-linear'):
        """
        Calculates the next state from the current state and the inputs using Euler integration on the dynamical model
        :param inputs: array or list containing the 4 inputs
        :param model: linear or non-linear, defaults to non-linear
        """
        if model == 'non-linear':
            state_derivative = np.zeros(12)
            state_derivative[0] = self.state[3]
            state_derivative[1] = self.state[4]
            state_derivative[2] = self.state[5]
            state_derivative[3] = inputs[0] / self.m * (
                    cos(self.state[8]) * sin(self.state[7]) +
                    cos(self.state[7]) * sin(self.state[6]) * sin(self.state[8])
            )
            state_derivative[4] = inputs[0] / self.m * (
                    sin(self.state[8]) * sin(self.state[7]) -
                    cos(self.state[8]) * cos(self.state[7]) * sin(self.state[6])
            )
            state_derivative[5] = inputs[0] / self.m * (
                    cos(self.state[6]) * cos(self.state[7])
            ) - constants.g
            state_derivative[6] = self.state[9]
            state_derivative[7] = self.state[10]
            state_derivative[8] = self.state[11]
            state_derivative[9] = (
                    ((-self.J[1, 1] + self.J[2, 2]) / self.J[0, 0]) * self.state[7] * self.state[8] +
                    inputs[1] / self.J[0, 0]
            )
            state_derivative[10] = (
                    ((self.J[0, 0] - self.J[2, 2]) / self.J[1, 1]) * self.state[6] * self.state[8] +
                    inputs[2] / self.J[1, 1]
            )
            state_derivative[11] = (
                    ((-self.J[0, 0] + self.J[1, 1]) / self.J[2, 2]) * self.state[6] * self.state[7] +
                    inputs[3] / self.J[2, 2]
            )

            self.state = self.state + state_derivative * constants.dt
        else:
            self.state = self.A @ self.state + self.B @ inputs + self.G

    # Function to display the drone in pygame
    def display(self, scr, colors, view_angles, origin, scale):

        # defines current position of rotors
        rotor_pos_wrt_drone = [np.dot(self.drone_rotation_matrix, [0.1, 0.1, 0]),
                               np.dot(self.drone_rotation_matrix, [-0.1, -0.1, 0]),
                               np.dot(self.drone_rotation_matrix, [-0.1, 0.1, 0]),
                               np.dot(self.drone_rotation_matrix, [0.1, -0.1, 0])]
        rotor_pos = [self.pos + rotor_pos_wrt_drone[0],
                     self.pos + rotor_pos_wrt_drone[1],
                     self.pos + rotor_pos_wrt_drone[2],
                     self.pos + rotor_pos_wrt_drone[3]]

        # draw drone facing direction
        pg.draw.line(scr, colors.cyan, projection(self.pos, view_angles, origin, scale),
                     projection(self.pos + np.dot(self.drone_rotation_matrix, [0.1, 0, 0]), view_angles, origin, scale),
                     5)

        # draw drone diagonals
        pg.draw.line(scr, colors.red, projection(rotor_pos[0], view_angles, origin, scale),
                     projection(rotor_pos[1], view_angles, origin, scale), 5)
        pg.draw.line(scr, colors.red, projection(rotor_pos[2], view_angles, origin, scale),
                     projection(rotor_pos[3], view_angles, origin, scale), 5)

        # draw drone rotors
        rotor_radius = 0.05
        for pos in rotor_pos:
            for theta in range(40):
                circle_pos = pos + np.dot(self.drone_rotation_matrix,
                                          np.array([sin(theta) * rotor_radius, cos(theta) * rotor_radius, 0]))
                projected_circle_pos = projection(circle_pos, view_angles, origin, scale)
                projected_circle_pos = [int(round(projected_circle_pos[0])), int(round(projected_circle_pos[1]))]
                pg.draw.circle(scr, colors.white, projected_circle_pos, 1)


# for testing only
x_current = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
x_target = [5, 1, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0]
quad = Drone([0, 0, 0, 0, 0, 0])

T = 30

NX = 12
NU = 4

x = np.zeros((T + 1, NX))
u = np.zeros((T, NU))
Y = np.ones((T, 6 + NU)) * x_target[:10]
yN = np.ones((1, 6)) * x_target[:6]
# Y = np.ones((T, 6 + NU)) * [1, 1, 0.5, 0, 0, 0, 0, 0, 0, 0]
# yN = np.ones((1, 6)) * [1, 1, 0.5, 0, 0, 0]
Q = np.diag([1, 1, 1, 1, 1, 1, 0.3, 0.3, 0.3, 0.3])
Qf = np.eye(6)

obstacles = np.array([
    [2, 2, 2]
])

t_start = time.time()

for i in range(1000):
    # u, x = solver.mpc(quad, quad.state, x_target)
    x, u = acado.mpc(0, 1, np.array([quad.state]), x, u, Y, yN, np.transpose(np.tile(Q, T)), Qf, 0, obstacles)
    # print('u', u[0])
    # print('x', x[:, :3])
    # print('y', x[1])
    # print('z', x[2])

    quad.update_state(u[0], model='non-linear')

    if i % 100 == 0:
        print('iteration', i)
        print('quad state', quad.state)
        print('')
        print('')

print('time', time.time() - t_start)
