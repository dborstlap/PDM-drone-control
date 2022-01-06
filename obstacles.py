""" 
name:     obstacles.py
authors:  Dries, Wesley, Tanya, Koen
function: This file contains functions and classes needed to represent and display obstacles
"""

drone_radius = 1
# ------------------------- IMPORTS --------------------------------

from math import sin, cos
import numpy as np
import random as rd
import pygame as pg
from functions import rotation_matrix, projection, depth_scale, pressed_keys, colors
import constants

# ---------------------------- METEORITE CLASS ---------------------------------
class Meteorite:
    # adds a moving
    def __init__(self, pos, vel, size):
        self.pos = np.array(pos)
        self.vel = np.array(vel)
        self.size = size

    def update_position(self, dt):
        self.pos = self.pos + self.vel * dt

    def display(self, scr, colors, view_angles, origin, scale):
        screen_pos = projection(self.pos, view_angles, origin, scale)
        pg.draw.circle(scr, colors.grey, screen_pos, self.size * scale * depth_scale(self.pos, view_angles, scale))

    def add_constraints(self, constraints, x, n, meteorite_slack):
        print(self.pos)
        print(self.vel)
        pos_expected = self.pos + constants.dt * n * self.vel
        constraints += [(x[0,n] - pos_expected[0])**2 + (x[1, n] - pos_expected[1])**2 + (x[2,n] - pos_expected[2])**2 >= (self.size + meteorite_slack + drone_radius)**2]

class Cuboid_obstacle:
    # Represents static cuboid obstacle
    def __init__(self, position, dimensions):
        self.pos = np.array(position)
        self.dimensions = dimensions
        self.cube_constraints = np.array([[self.pos[0] * -1 + self.dimensions[0]/2],
                                          [self.pos[0] + self.dimensions[0] / 2],
                                          [self.pos[1] * -1 + self.dimensions[1]/2],
                                          [self.pos[1] + self.dimensions[1] / 2],
                                          [self.pos[2] * -1 + self.dimensions[2]/2],
                                          [self.pos[2] + self.dimensions[2]/2]])

    def add_constraints(self, x, n, constraints, margin, obstacle_binary, cuboid_slack, j):

        constraints += [self.cube_constraints[0] - x[0, n] + cuboid_slack[0 + j, n] >= margin * obstacle_binary[0 + j, n] + drone_radius]
        constraints += [x[0, n] - self.cube_constraints[1] + cuboid_slack[1 + j, n] >= margin * obstacle_binary[1 + j, n] + drone_radius]
        constraints += [self.cube_constraints[2] - x[1, n] + cuboid_slack[2 + j, n] >= margin * obstacle_binary[2 + j, n] + drone_radius]
        constraints += [x[1, n] - self.cube_constraints[3] + cuboid_slack[3 + j, n] >= margin * obstacle_binary[3 + j, n] + drone_radius]
        constraints += [self.cube_constraints[4] - x[2, n] + cuboid_slack[4 + j, n] >= margin * obstacle_binary[4 + j, n] + drone_radius]
        constraints += [x[2, n] - self.cube_constraints[5] + cuboid_slack[5 + j, n] >= margin * obstacle_binary[5 + j, n] + drone_radius]
        # ensure that at least one binary variable is set to zero for the obstacle, i.e. the obstacle is avoided for at
        # least one plane of the obstacle
        constraints += [np.sum(obstacle_binary[j:j+6, n], axis=0) <= 5]

#        self.dimensions = margin *(np.array(dimensions)+drone_radius)
#        self.constraints = np.array([[self.pos[0] + self.dimensions[0]/2],
#                                [self.pos[1] + self.dimensions[1]/2],
#                                [self.pos[2] + self.dimensions[2]/2],
#                                [self.pos[0]*-1 + self.dimensions[0]/2],
#                                [self.pos[1]*-1 + self.dimensions[1]/2],
#                                [self.pos[2]*-1 + self.dimensions[2]/2]]).reshape((6,))

 #       self.P = np.vstack((np.eye(3), -np.eye(3)))


# ---------------------------- CUBOID (=prism) CLASS ---------------------------------
class Cuboid:
    # this part sets all the points x,y,x co-cords at the correct locations
    #  _____   7____6
    # |\____\  4____5
    # |z|    | 3____2
    # y\|____| 0____1
    #     x
    def __init__(self, dimensions):  # vertices must be numpy array
        self.dimensions = dimensions
        self.vertices = np.array(
            [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]]) * self.dimensions
        self.edges = ((0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7))

    def display(self, scr, colors, view_angles, origin, scale):
        for edge in self.edges:
            pg.draw.line(scr, colors.white,
                         projection(self.vertices[edge[0]], view_angles, origin, scale),
                         projection(self.vertices[edge[1]], view_angles, origin, scale), 4)

    def display(self, scr, colors, view_angles, origin, scale):
        screen_pos = projection(self.pos, view_angles, origin, scale)
        pg.draw.circle(scr, colors.grey, screen_pos, self.size * scale * depth_scale(self.pos, view_angles, scale))
