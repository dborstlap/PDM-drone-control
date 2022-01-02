""" 
name:     obstacles.py
authors:  Dries, Wesley, Tanya, Koen
function: This file contains functions and classes needed to represent and display obstacles
"""

# ------------------------- IMPORTS --------------------------------

from math import sin, cos
import numpy as np
import random as rd
import pygame as pg

from functions import rotation_matrix, projection, depth_scale, pressed_keys, colors

# ---------------------------- METEORITE CLASS ---------------------------------
class Meteorite:
    def __init__(self, pos, vel, size):
        self.pos = np.array(pos)
        self.vel = np.array(vel)
        self.size = size

    def update_position(self, dt):
        self.pos[-1] = self.pos[-1] - self.vel[-1] * dt

    def display(self, scr, colors, view_angles, origin, scale):
        screen_pos = projection(self.pos, view_angles, origin, scale)
        pg.draw.circle(scr, colors.grey, screen_pos, self.size * scale * depth_scale(self.pos, view_angles, scale))


# ---------------------------- Obstacle CLASS ---------------------------------

class Obstacle:
    # Represents static cuboid obstacle
    def __init__(self, position, dimensions, safetyfactor, drone_radius):
        self.pos = np.array(position)
        self.visual_dimensions = dimensions
        self.dimensions = safetyfactor*(np.array(dimensions)+drone_radius)
        self.constraints = np.array([[self.pos[0] + self.dimensions[0]/2],
                                [self.pos[1] + self.dimensions[1]/2],
                                [self.pos[2] + self.dimensions[2]/2],
                                [self.pos[0]*-1 + self.dimensions[0]/2],
                                [self.pos[1]*-1 + self.dimensions[1]/2],
                                [self.pos[2]*-1 + self.dimensions[2]/2]]).reshape((6,))

        self.P = np.vstack((np.eye(3), -np.eye(3)))

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
