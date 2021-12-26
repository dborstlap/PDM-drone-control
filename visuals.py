""" 
name:     visuals.py
authors:  Dries, Wesley, Tanya, Koen
function: Contains the drone model
"""

#---------------------------- IMPORTS ---------------------------------

from math import sin,cos
import numpy as np
import random as rd
import pygame as pg

from functions import rotation_matrix, projection, depth_scale, pressed_keys, colors


#---------------------------- AXES ---------------------------------













#---------------------------- EXPLOSION ---------------------------------
explosion_images = []
for num in range(1, 6):
	img = pg.image.load(f"explosion/exp"+str(num)+".png")
	explosion_images.append(img)

def display_explosion(scr, scale, colission, pos, dt, localtime, view_angles, origin):
    if colission:
        localtime = 0
        colission = 0
    localtime += dt      

    if int(localtime) < 5:
        img = explosion_images[int(localtime)]
        img = pg.transform.scale(img, (scale/10, scale/10))
        pos_screen = projection(pos,view_angles,origin,scale)
        rect = img.get_rect()
        rect.center = [pos_screen[0], pos_screen[1]]
        scr.blit(img, rect)
    return localtime



#---------------------------- CUBOID (=prism) CLASS ---------------------------------
class Cuboid:
    #this part sets all the points x,y,x co-cords at the correct locations
    #  _____   7____6
    # |\____\  4____5
    # |z|    | 3____2
    # y\|____| 0____1
    #     x
    def __init__(self, dimensions): # vertices must be numpy array
        self.dimensions = dimensions
        self.vertices = np.array([[0,0,0], [1,0,0], [1,1,0], [0,1,0], [0,0,1], [1,0,1], [1,1,1], [0,1,1]]) * self.dimensions
        self.edges = ((0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7))

    def display(self, scr, colors, view_angles, origin, scale):
        for edge in self.edges:
            pg.draw.line(scr, colors.white, 
                         projection(self.vertices[edge[0]],view_angles,origin,scale), 
                         projection(self.vertices[edge[1]],view_angles,origin,scale), 4)










