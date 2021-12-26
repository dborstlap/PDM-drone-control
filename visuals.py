""" 
name:     visuals.py
authors:  Dries, Wesley, Tanya, Koen
function: Contains the drone model
"""

# ---------------------------- IMPORTS ---------------------------------

from math import sin, cos
import numpy as np
import random as rd
import pygame as pg

from functions import rotation_matrix, projection, depth_scale, pressed_keys, colors

#---------------------------- EXPLOSION ---------------------------------
explosion_images = []
for num in range(1, 6):
    img = pg.image.load(f"explosion/exp" + str(num) + ".png")
    explosion_images.append(img)


def display_explosion(scr, scale, colission, pos, dt, localtime, view_angles, origin):
    if colission:
        localtime = 0
        colission = 0
    localtime += dt

    if int(localtime) < 5:
        img = explosion_images[int(localtime)]
        img = pg.transform.scale(img, (int(scale / 10), int(scale / 10)))
        pos_screen = projection(pos, view_angles, origin, scale)
        rect = img.get_rect()
        rect.center = [pos_screen[0], pos_screen[1]]
        scr.blit(img, rect)
    return localtime
