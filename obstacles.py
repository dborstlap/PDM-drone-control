""" 
name:     obstacles.py
authors:  Dries, Wesley, Tanya, Koen
function: This file contains functions and classes needed to represent and display obstacles
"""

# ------------------------- IMPORTS --------------------------------

import numpy as np
import pygame as pg

import constants
from functions import projection, depth_scale, colors


# ---------------------------- METEORITE CLASS ---------------------------------
class Meteorite:
    # adds a moving obstacle
    def __init__(self, pos, vel, size):
        """

        :param pos: x, y, z
        :param vel: x_dot, y_dot, z_dot
        :param size: radius
        """
        self.pos = np.array(pos)
        self.vel = np.array(vel)
        self.size = size

    def update_position(self, dt):
        self.pos = self.pos + self.vel * dt

    def display(self, scr, colors, view_angles, origin, scale):
        screen_pos = projection(self.pos, view_angles, origin, scale)
        pg.draw.circle(scr, colors.grey, screen_pos, self.size * scale * depth_scale(self.pos, view_angles, scale))

    def add_constraints(self, x, n, constraints, margin, moving_obstacle_binary, moving_obstacle_slack, j):
        pos_expected = self.pos + constants.dt * n * self.vel
        cube_constraints = np.array([[pos_expected[0] - self.size],
                                     [pos_expected[0] + self.size],
                                     [pos_expected[1] - self.size],
                                     [pos_expected[1] + self.size],
                                     [pos_expected[2] - self.size],
                                     [pos_expected[2] + self.size]])

        constraints += [
            cube_constraints[0] - x[0, n] + moving_obstacle_slack[0 + j, n] - constants.quadrotor_size
            >= margin * moving_obstacle_binary[
                0 + j, n] + 1]
        constraints += [
            x[0, n] - cube_constraints[1] + moving_obstacle_slack[1 + j, n] - constants.quadrotor_size
            >= margin * moving_obstacle_binary[
                1 + j, n] + 1]
        constraints += [
            cube_constraints[2] - x[1, n] + moving_obstacle_slack[2 + j, n] - constants.quadrotor_size
            >= margin * moving_obstacle_binary[
                2 + j, n] + 1]
        constraints += [
            x[1, n] - cube_constraints[3] + moving_obstacle_slack[3 + j, n] - constants.quadrotor_size
            >= margin * moving_obstacle_binary[
                3 + j, n] + 1]
        constraints += [
            cube_constraints[4] - x[2, n] + moving_obstacle_slack[4 + j, n] - constants.quadrotor_size
            >= margin * moving_obstacle_binary[
                4 + j, n] + 1]
        constraints += [
            x[2, n] - cube_constraints[5] + moving_obstacle_slack[5 + j, n] - constants.quadrotor_size
            >= margin * moving_obstacle_binary[
                5 + j, n] + 1]
        # ensure that at least one binary variable is set to zero for the obstacle, i.e. the obstacle is avoided for at
        # least one plane of the obstacle
        constraints += [
            moving_obstacle_binary[0 + j, n] + moving_obstacle_binary[1 + j, n] + moving_obstacle_binary[2 + j, n]
            + moving_obstacle_binary[3 + j, n] + moving_obstacle_binary[4 + j, n] + moving_obstacle_binary[
                5 + j, n] <= 5]



class Cuboid:
    # this part sets all the points x,y,x co-cords at the correct locations
    #  _____   7____6
    # |\____\  4____5
    # |z|    | 3____2
    # y\|____| 0____1
    #     x
    def __init__(self, dim=[1,1,1], pos=[0,0,0], edge_color=colors.white, face_color=colors.white):  # vertices must be numpy array
        self.dim = np.array(dim)
        self.pos = np.array(pos)
        self.vertices = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]]) * self.dim + self.pos
        self.edges = (self.vertices[[0, 1]], 
                      self.vertices[[1, 2]], 
                      self.vertices[[2, 3]], 
                      self.vertices[[3, 0]], 
                      self.vertices[[4, 5]], 
                      self.vertices[[5, 6]], 
                      self.vertices[[6, 7]], 
                      self.vertices[[7, 4]], 
                      self.vertices[[0, 4]], 
                      self.vertices[[1, 5]], 
                      self.vertices[[2, 6]], 
                      self.vertices[[3, 7]])
        self.faces = [self.vertices[[0,1,2,3]], 
                      self.vertices[[4,5,6,7]], 
                      self.vertices[[0,1,5,4]], 
                      self.vertices[[2,3,7,6]], 
                      self.vertices[[0,3,7,4]], 
                      self.vertices[[1,2,6,5]]]
        self.edge_color=edge_color
        self.face_color=face_color
        self.cube_constraints = np.array([[self.pos[0]],
                                          [self.pos[0] + self.dim[0]],
                                          [self.pos[1]],
                                          [self.pos[1] + self.dim[1]],
                                          [self.pos[2]],
                                          [self.pos[2] + self.dim[2]]])
        
    def add_constraints(self, x, n, constraints, margin, obstacle_binary, cuboid_slack, j):
        constraints += [
            self.cube_constraints[0] - x[0, n] + cuboid_slack[0 + j, n] - constants.quadrotor_size
            >= margin * obstacle_binary[0 + j, n]]
        constraints += [
            x[0, n] - self.cube_constraints[1] + cuboid_slack[1 + j, n] - constants.quadrotor_size
            >= margin * obstacle_binary[1 + j, n]]
        constraints += [
            self.cube_constraints[2] - x[1, n] + cuboid_slack[2 + j, n] - constants.quadrotor_size
            >= margin * obstacle_binary[2 + j, n]]
        constraints += [
            x[1, n] - self.cube_constraints[3] + cuboid_slack[3 + j, n] - constants.quadrotor_size
            >= margin * obstacle_binary[3 + j, n]]
        constraints += [
            self.cube_constraints[4] - x[2, n] + cuboid_slack[4 + j, n] - constants.quadrotor_size
            >= margin * obstacle_binary[4 + j, n]]
        constraints += [
            x[2, n] - self.cube_constraints[5] + cuboid_slack[5 + j, n] - constants.quadrotor_size
            >= margin * obstacle_binary[5 + j, n]]
        # ensure that at least one binary variable is set to zero for the obstacle, i.e. the obstacle is avoided for at
        # least one plane of the obstacle
        constraints += [obstacle_binary[0 + j, n] + obstacle_binary[1 + j, n] + obstacle_binary[2 + j, n]
                        + obstacle_binary[3 + j, n] + obstacle_binary[4 + j, n] + obstacle_binary[5 + j, n] <= 5]
        
    
    def display(self, scr, colors, view_angles, origin, scale, wireframe=False):
        if wireframe == False:
            for face in self.faces:
                points = [projection(point, view_angles, origin, scale) for point in face]
                pg.draw.polygon(scr, self.face_color, points)
        
        for edge in self.edges:
            pg.draw.line(scr, self.edge_color, projection(edge[0], view_angles, origin, scale), projection(edge[1], view_angles, origin, scale), 4)

    