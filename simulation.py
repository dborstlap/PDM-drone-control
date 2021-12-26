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
from model import drone
from obstacles import meteorite, cuboid
from visuals import display_explosion

# ------------------------- CLASSES --------------------------------

# DRONE CLASS
#class drone:
#    # Initialize some drone parameters
#    def __init__(self, X):
#        self.X = np.array(X)
#        self.pos = np.array([X[0],X[1],X[2]])
#        self.drone_rotation_matrix = np.eye(3)

#    # Update the position (and state space?) of the drone
#    def update_position(self):
#        self.X = self.X

#    # Function to display the drone in pygame
#    def display(self):

#        # defines current position of rotors
#        rotor_pos_wrt_drone = [np.dot(self.drone_rotation_matrix,[0.1,0.1,0]),
#                               np.dot(self.drone_rotation_matrix,[-0.1,-0.1,0]),
#                               np.dot(self.drone_rotation_matrix,[-0.1,0.1,0]),
#                               np.dot(self.drone_rotation_matrix,[0.1,-0.1,0])]
#        rotor_pos = [self.pos + rotor_pos_wrt_drone[0],
#                     self.pos + rotor_pos_wrt_drone[1],
#                     self.pos + rotor_pos_wrt_drone[2],
#                     self.pos + rotor_pos_wrt_drone[3]]

#        # draw drone facing direction
#        pg.draw.line(scr, colors.cyan, projection(self.pos,view_angles,origin,scale), 
#                                      projection(self.pos+np.dot(self.drone_rotation_matrix,[0.1,0,0]),view_angles,origin,scale), 5)
  
#        # draw drone diagonals
#        pg.draw.line(scr, colors.red, projection(rotor_pos[0],view_angles,origin,scale), projection(rotor_pos[1],view_angles,origin,scale), 5)
#        pg.draw.line(scr, colors.red, projection(rotor_pos[2],view_angles,origin,scale), projection(rotor_pos[3],view_angles,origin,scale), 5)

#        # draw drone rotors
#        rotor_radius = 0.05
#        for pos in rotor_pos:
#            for theta in range(40):
#                circle_pos = pos+np.dot(self.drone_rotation_matrix,np.array([sin(theta)*rotor_radius,cos(theta)*rotor_radius,0]))
#                projected_circle_pos = projection(circle_pos,view_angles,origin,scale)
#                projected_circle_pos = [int(round(projected_circle_pos[0])),int(round(projected_circle_pos[1]))]
#                pg.draw.circle(scr, colors.white, projected_circle_pos, 1)


## METEORITE CLASS
#class meteorite:
#    def __init__(self, pos, vel, size):
#        self.pos = np.array(pos)
#        self.vel = np.array(vel)
#        self.size = size
       

#    def update_position(self):
#        self.pos[-1] = self.pos[-1] - self.vel[-1]*dt


#    def display(self):
#        screen_pos = projection(self.pos,view_angles,origin,scale)
#        pg.draw.circle(scr, colors.grey, screen_pos, self.size*scale*depth_scale(self.pos,view_angles, scale))


## CUBOID (=prism) CLASS
#class cuboid:
#    #this part sets all the points x,y,x co-cords at the correct locations
#    #  _____   7____6
#    # |\____\  4____5
#    # |z|    | 3____2
#    # y\|____| 0____1
#    #     x
#    def __init__(self, dimensions): # vertices must be numpy array
#        self.dimensions = dimensions
#        self.vertices = np.array([[0,0,0], [1,0,0], [1,1,0], [0,1,0], [0,0,1], [1,0,1], [1,1,1], [0,1,1]]) * self.dimensions
#        self.edges = ((0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7))

#    def display(self):
#        for edge in self.edges:
#            pg.draw.line(scr, colors.white, 
#                         projection(self.vertices[edge[0]],view_angles,origin,scale), 
#                         projection(self.vertices[edge[1]],view_angles,origin,scale), 4)


#explosion_images = []
#for num in range(1, 6):
#	img = pg.image.load(f"explosion/exp"+str(num)+".png")
#	explosion_images.append(img)


#def display_explosion(scale, colission, pos, dt, localtime):
#    if colission:
#        localtime = 0
#        colission = 0
#    localtime += dt      

#    if int(localtime) < 5:
#        img = explosion_images[int(localtime)]
#        img = pg.transform.scale(img, (scale/10, scale/10))
#        pos_screen = projection(pos,view_angles,origin,scale)
#        rect = img.get_rect()
#        rect.center = [pos_screen[0], pos_screen[1]]
#        scr.blit(img, rect)
#    return localtime



#################################################### MAIN ####################################################


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
bbox = cuboid([15,10,10])
meteorite_frequency = 5   # amout of meteorites per second that are created
colission = 0
localtime = 100

# ------------------------------ RUN LOOP ---------------------------------------
tprev = pg.time.get_ticks()*0.001
running = True
while running:
    time = pg.time.get_ticks()*0.001
    dt = time-tprev
    tprev = time

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
        meteorites.append(meteorite([rd.uniform(0, bbox.dimensions[0]), rd.uniform(0, bbox.dimensions[1]), bbox.dimensions[2]],
                                    [0, 0, rd.uniform(1.,2.)], 
                                     rd.uniform(0.2,0.5)))
        
    for i,m in enumerate(meteorites):
        meteorites[i].update_position(dt)
        if m.pos[2] < 0: meteorites.pop(i)

    #--------------------------- drone ---------------------------
    X = [5,5,5,0,0,0]
    drone1 = drone(X)


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












