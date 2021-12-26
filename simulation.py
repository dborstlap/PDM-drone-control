""" 
name:     simulation.py
authors:  Dries, Wesley, Tanya, Koen
function: This file can simulate and visualize the drone in pygame 
"""

# ------------------------- IMPORTS --------------------------------

from math import sin, cos
import numpy as np
import random as rd
import pygame as pg
pg.init()

from functions import rotation_matrix, projection, depth_scale, pressed_keys, colors
from model import Drone
from obstacles import Meteorite
from visuals import display_explosion, Cuboid


# -------------------------- VARIABLES -------------------------------------------
fullscreen = False # full screen does not seem to work accurately in pygame??
screensize = np.array([1280,720])     # if not fullscreen

if fullscreen: scr = pg.display.set_mode((0, 0), pg.FULLSCREEN)
if not fullscreen: scr = pg.display.set_mode((screensize[0],screensize[1]))


scale = 1000
origin = np.array([100,screensize[1]-100])        # w.r.t. left upper corner of screen, should be np.array([screensize[0]/2,screensize[0]/2]) for MACHINE VISION to be centered
#view_angles = np.array([0.0, 0.0, 0.0])    # viewing angles of general coordinate frame, should be np.array([0,0,0]) for MACHINE VISION
view_angles = np.array([np.pi/2, 0.0, 0.0])    # viewing angles of general coordinate frame, should be np.array([0,0,0]) for MACHINE VISION

meteorites = []
meteorite_counter = 0
mouse_position = pg.mouse.get_pos()
text_fonts = [pg.font.SysFont('HELVETICA', i, bold=True, italic=False) for i in range(40)]
bbox = Cuboid([15,10,10])
meteorite_frequency = 5   # amout of meteorites per second that are created
colission = 0
localtime = 100

real_time = True          # If TRUE, simulation runs in real time. If FALSE, simulation will run as fast as possible (as fast as computer can do the calculations)
dt = 0.1                  # delta t, time interval per iteration

# ------------------------------ RUN LOOP ---------------------------------------
tprev = pg.time.get_ticks()*0.001
running = True
while running:
    time = pg.time.get_ticks()*0.001
    dt_real = time-tprev
    tprev = time

    time_difference = int((dt-dt_real)*1000) # how much faster the computer computes the iteration then the defined dt per iteration (in µs)
    if time_difference > 0 and real_time == True:
        pg.time.delay(time_difference)

    # ----------- 3D mouse controls ---------------
    events = pg.event.get()
    for event in events:
        if event.type == pg.MOUSEBUTTONDOWN:
            if event.button == 4:
                scale += 0.1*scale
                origin = origin + 0.1*(origin - np.array(pg.mouse.get_pos()))
            if event.button == 5:
                scale -= 0.1*scale
                origin = 0.9*origin+0.1*np.array(pg.mouse.get_pos())

    previous_mouse_position = mouse_position
    mouse_position = pg.mouse.get_pos()
    mouse_clicks = pg. mouse. get_pressed()
    if mouse_clicks[0] == 1:
        origin = origin+(np.array(mouse_position)-np.array(previous_mouse_position))
    if mouse_clicks[2] == 1:
        mouse_movement = np.array(mouse_position)-np.array(previous_mouse_position)

        something = np.dot([[1, 0, 0], [0, 1, 0]], rotation_matrix(view_angles))
        view_angles += np.dot(np.dot(rotation_matrix(view_angles), [0.01*mouse_movement[1], -0.01*mouse_movement[0], 0]), rotation_matrix(view_angles))


    #--------------------------- meteorites ---------------------------
    while meteorite_counter < int(time*meteorite_frequency):
        meteorite_counter += 1
        meteorites.append(Meteorite([rd.uniform(0, bbox.dimensions[0]), rd.uniform(0, bbox.dimensions[1]), bbox.dimensions[2]],
                                    [0, 0, rd.uniform(1.,2.)], 
                                     rd.uniform(0.2,0.5)))
        
    for i,m in enumerate(meteorites):
        meteorites[i].update_position(dt)
        if m.pos[2] < 0: meteorites.pop(i)

    #--------------------------- drone ---------------------------
    X = [5,5,5,0,0,0]
    drone1 = Drone(X)


    # ---------- DISPLAY STUFF -------------------
    # background
    scr.fill(colors.black)


    # explosion
    for i,m in enumerate(meteorites):
        if np.linalg.norm(drone1.pos-m.pos) < 1:
            meteorites.pop(i)
            colission = True
        localtime = display_explosion(scr, scale, colission, drone1.pos, dt, localtime, view_angles, origin)
        colission = False



    # meteorites
    [m.display(scr, colors, view_angles, origin, scale) for m in meteorites]

    # drone
    drone1.display(scr, colors, view_angles, origin, scale)

    # boundary box
    bbox.display(scr, colors, view_angles, origin, scale)

    # draw axis of global coordinate system
    pg.draw.line(scr, colors.lime, projection([0,0,0],view_angles,origin,scale), projection([1,0,0],view_angles,origin,scale), 4)
    pg.draw.line(scr, colors.lime, projection([0,0,0],view_angles,origin,scale), projection([0,1,0],view_angles,origin,scale), 4)
    pg.draw.line(scr, colors.lime, projection([0,0,0],view_angles,origin,scale), projection([0,0,1],view_angles,origin,scale), 4)

    text_font = text_fonts[20]
    x_axis_text, y_axis_text, z_axis_text = text_font.render('X', True, colors.lightblue), text_font.render('Y', True, colors.lightblue), text_font.render('Z', True, colors.lightblue)
    scr.blit(x_axis_text,x_axis_text.get_rect(center = projection([1.1,0,0],view_angles,origin,scale)))
    scr.blit(y_axis_text,y_axis_text.get_rect(center = projection([0,1.1,0],view_angles,origin,scale)))
    scr.blit(z_axis_text,z_axis_text.get_rect(center = projection([0,0,1.1],view_angles,origin,scale)))

    # update display
    pg.display.flip()
    # exit pygame
    for event in events:
        if event.type == pg.QUIT:
            pg.quit()
            pg.font.quit()
            running = False












