""" 
name:     model.py
authors:  Dries, Wesley, Tanya, Koen
function: Contains the drone model
"""

# ---------------------------- IMPORTS ---------------------------------

from math import sin, cos
import numpy as np
import random as rd
import pygame as pg

import constants
from functions import rotation_matrix, projection, depth_scale, pressed_keys, colors


l = 0.10
omega_max = 2000.0
k_F = 6.11 * 10**-8
k_M = 1.5 * 10**-9


# --------------------------- DRONE CLASS ---------------------------------
class Drone:
    m = 1
    J = np.array([
        [1.1, 0, 0],
        [0, 1, 0],
        [0, 0, 1.7]
    ]) * 10 ** (-3)

    A = np.eye(12) + np.array([
        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, -constants.g, 0, 0, 0, 0],
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
        [0, 1/J[0, 0], 0, 0],
        [0, 0, 1/J[1, 1], 0],
        [0, 0, 0, 1/J[2, 2]]
    ]) * constants.dt

    C = np.eye(6)

    D = np.zeros((6, 4))

    u_min = np.array([
        0,
        -l * k_F * omega_max ** 2,
        -l * k_F * omega_max ** 2,
        -2 * k_M * omega_max ** 2
    ])
    u_max = np.array([
        4 * k_F * omega_max ** 2,
        l * k_F * omega_max ** 2,
        l * k_F * omega_max ** 2,
        2 * k_M * omega_max ** 2
    ])

    # Initialize some drone parameters
    def __init__(self, X):
        self.X = np.array(X)
        self.pos = np.array([X[0], X[1], X[2]])
        self.drone_rotation_matrix = np.eye(3)

    # Update the position (and state space?) of the drone
    def update_position(self):
        self.X = self.X

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
