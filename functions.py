########### This file contains some functions used for the drone simulation ###############################

import pygame as pg
import numpy as np
from math import cos,sin




def rotation_matrix(angles):
    theta = angles[0]    # pitch  (x-axis)
    sigma = angles[1]    # roll   (y-axis)
    psi   = angles[2]    # yaw    (z-axis)

    X_rotation = np.array([[  1         ,  0         ,  0          ],
                           [  0         ,  cos(theta), -sin(theta) ],
                           [  0         ,  sin(theta),  cos(theta) ]])

    Y_rotation = np.array([[  cos(sigma),  0         ,  sin(sigma) ],
                           [  0         ,  1         ,  0          ],
                           [ -sin(sigma),  0         ,  cos(sigma) ]])

    Z_rotation = np.array([[  cos(psi)  , -sin(psi)  ,  0          ],
                           [  sin(psi)  ,  cos(psi)  ,  0          ],
                           [  0         ,  0         ,  1          ]])

    return np.dot(np.dot(Z_rotation, Y_rotation), X_rotation)



def projection(point,view_angles,origin,scale):
    rotated_point = np.dot(rotation_matrix(view_angles), point)
    projection_matrix = [[1, 0, 0],
                         [0, 1, 0]]
    return scale * depth_scale(point,view_angles, scale) * np.dot(projection_matrix,rotated_point) + origin


def depth_scale(point,view_angles, scale):
    rotated_point = np.dot(rotation_matrix(view_angles), point)
    return 1 / (30+rotated_point[2])



def pressed_keys():
    pressed_key = pg.key.get_pressed()
    keys = []
    if pressed_key[pg.K_UP]:
      keys.append('up')
    if pressed_key[pg.K_RIGHT]:
      keys.append('right')
    if pressed_key[pg.K_DOWN]:
      keys.append('down')
    if pressed_key[pg.K_LEFT]:
      keys.append('left')
    if pressed_key[pg.K_w]:
      keys.append('w')
    if pressed_key[pg.K_d]:
      keys.append('d')
    if pressed_key[pg.K_s]:
      keys.append('s')
    if pressed_key[pg.K_a]:
      keys.append('a')
    if pressed_key[pg.K_q]:
      keys.append('q')
    if pressed_key[pg.K_e]:
      keys.append('e')
    return keys

class colors:
  def __init__(self):
    self.white = (255,255,255)
    self.lightgrey = (208,208,208)
    self.grey = (138,138,138)
    self.black = (59,59,64)
    self.brown = (156,113,83)
    self.red = (181,64,64)
    self.orange = (234,173,83)
    self.yellow = (234,234,77)
    self.lime = (149,218,65)
    self.green = (100,129,56)
    self.cyan = (57,123,149)
    self.lightblue = (150,185,234)
    self.blue = (60,109,181)
    self.purple = (189,124,221)
    self.magenta = (225,143,218)
    self.pink = (240,181,211)
color = colors()
