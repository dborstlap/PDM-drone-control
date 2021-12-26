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
