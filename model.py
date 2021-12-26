""" 
name:     model.py
authors:  Dries, Wesley, Tanya, Koen
function: Contains the drone model
"""

#---------------------------- IMPORTS ---------------------------------

from math import sin,cos
import numpy as np
import random as rd
import pygame as pg

from functions import rotation_matrix, projection, depth_scale, pressed_keys, colors

#--------------------------- DRONE CLASS ---------------------------------
class Drone:
    # Initialize some drone parameters
    def __init__(self, X):
        self.X = np.array(X)
        self.pos = np.array([X[0],X[1],X[2]])
        self.drone_rotation_matrix = np.eye(3)

    # Update the position (and state space?) of the drone
    def update_position(self):
        self.X = self.X

    # Function to display the drone in pygame
    def display(self, scr, colors, view_angles, origin, scale):

        # defines current position of rotors
        rotor_pos_wrt_drone = [np.dot(self.drone_rotation_matrix,[0.1,0.1,0]),
                               np.dot(self.drone_rotation_matrix,[-0.1,-0.1,0]),
                               np.dot(self.drone_rotation_matrix,[-0.1,0.1,0]),
                               np.dot(self.drone_rotation_matrix,[0.1,-0.1,0])]
        rotor_pos = [self.pos + rotor_pos_wrt_drone[0],
                     self.pos + rotor_pos_wrt_drone[1],
                     self.pos + rotor_pos_wrt_drone[2],
                     self.pos + rotor_pos_wrt_drone[3]]

        # draw drone facing direction
        pg.draw.line(scr, colors.cyan, projection(self.pos,view_angles,origin,scale), 
                                      projection(self.pos+np.dot(self.drone_rotation_matrix,[0.1,0,0]),view_angles,origin,scale), 5)
  
        # draw drone diagonals
        pg.draw.line(scr, colors.red, projection(rotor_pos[0],view_angles,origin,scale), projection(rotor_pos[1],view_angles,origin,scale), 5)
        pg.draw.line(scr, colors.red, projection(rotor_pos[2],view_angles,origin,scale), projection(rotor_pos[3],view_angles,origin,scale), 5)

        # draw drone rotors
        rotor_radius = 0.05
        for pos in rotor_pos:
            for theta in range(40):
                circle_pos = pos+np.dot(self.drone_rotation_matrix,np.array([sin(theta)*rotor_radius,cos(theta)*rotor_radius,0]))
                projected_circle_pos = projection(circle_pos,view_angles,origin,scale)
                projected_circle_pos = [int(round(projected_circle_pos[0])),int(round(projected_circle_pos[1]))]
                pg.draw.circle(scr, colors.white, projected_circle_pos, 1)










