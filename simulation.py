""" 
name:     simulation.py
authors:  Dries, Wesley, Tanya, Koen
function: This file can simulate and visualize the drone in pygame 
"""
# ------------------------- IMPORTS --------------------------------

import numpy as np
import pygame as pg

import acado
import constants
from scenario import default_obstacle_set, four_meteorites

pg.init()

from functions import rotation_matrix, projection, colors, make_video
from model import Drone
from obstacles import Cuboid

# -------------------------- VARIABLES -------------------------------------------
x_target = [5, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
x_current = [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]
quad = Drone(x_current)

meteorites = four_meteorites()

T = 40
NX = 13
NU = 4
n_obstacles = 6

x = np.zeros((T + 1, NX))
u = np.zeros((T, NU + n_obstacles))
Y = np.ones((T, 6 + NU + n_obstacles)) * np.hstack((x_target[:6], np.repeat(0, NU + n_obstacles)))
yN = np.ones((1, 6)) * x_target[:6]
Q_x = 1
Q_u = 0.9
Q_o = 10000
Q = np.diag([Q_x, Q_x, Q_x, Q_x, Q_x, Q_x, Q_u, Q_u, Q_u, Q_u, Q_o, Q_o, Q_o, Q_o, Q_o, Q_o])
Qf = np.eye(6)


fullscreen = False # full screen does not seem to work accurately in pygame??
screensize = np.array([1280,720])     # if not fullscreen

if fullscreen: scr = pg.display.set_mode((0, 0), pg.FULLSCREEN)
if not fullscreen: scr = pg.display.set_mode((screensize[0],screensize[1]))

scale = 1700
origin = np.array([100, screensize[1]-100])        # w.r.t. left upper corner of screen, should be np.array([screensize[0]/2,screensize[0]/2]) for MACHINE VISION to be centered
view_angles = np.array([np.pi/2, 0.0, 0.0])    # viewing angles of general coordinate frame, should be np.array([0,0,0]) for MACHINE VISION

obstacle1 = Cuboid(dim=[2,10,2], pos = [7,0,0], edge_color=colors.blue, face_color=colors.light_blue)

meteorite_counter = 0
mouse_position = pg.mouse.get_pos()
text_fonts = [pg.font.SysFont('HELVETICA', i, bold=True, italic=False) for i in range(40)]
bbox = Cuboid(dim=[15,10,10])
meteorite_frequency = 5   # amout of meteorites per second that are created
collision = 0
localtime = 100

real_time = True          # If TRUE, simulation runs in real time. If FALSE, simulation will run as fast as possible (as fast as computer can do the calculations)
dt = constants.dt         # delta t, time interval per iteration
loop_number = 0           # every loop, 1 will be added

# ------------------------------ RUN LOOP ---------------------------------------
tprev = pg.time.get_ticks()*0.001
running = True

while running:

    # ---------- time -------------------
    loop_number += 1
    time = pg.time.get_ticks()*0.001
    dt_real = time-tprev
    tprev = time

    time_difference = int((dt-dt_real) * 1) # how much faster the computer computes the iteration then the defined dt per iteration (in µs)
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


    #--------------------------- background and axes ---------------------------
    # background
    scr.fill(colors.black)

    # boundary box
    bbox.display(scr, colors, view_angles, origin, scale, wireframe=True)

    # draw axis of global coordinate system
    pg.draw.line(scr, colors.lime, projection([0,0,0],view_angles,origin,scale), projection([1,0,0],view_angles,origin,scale), 4)
    pg.draw.line(scr, colors.lime, projection([0,0,0],view_angles,origin,scale), projection([0,1,0],view_angles,origin,scale), 4)
    pg.draw.line(scr, colors.lime, projection([0,0,0],view_angles,origin,scale), projection([0,0,1],view_angles,origin,scale), 4)

    text_font = text_fonts[20]
    x_axis_text, y_axis_text, z_axis_text = text_font.render('X', True, colors.light_blue), text_font.render('Y', True, colors.light_blue), text_font.render('Z', True, colors.light_blue)
    scr.blit(x_axis_text,x_axis_text.get_rect(center = projection([1.1,0,0],view_angles,origin,scale)))
    scr.blit(y_axis_text,y_axis_text.get_rect(center = projection([0,1.1,0],view_angles,origin,scale)))
    scr.blit(z_axis_text,z_axis_text.get_rect(center = projection([0,0,1.1],view_angles,origin,scale)))

    #---------------- meteorites ------------------------
    #while meteorite_counter < int(loop_number*dt*meteorite_frequency):
    #    meteorite_counter += 1
    #    meteorites.append(Meteorite([rd.uniform(0, bbox.dimensions[0]), rd.uniform(0, bbox.dimensions[1]), bbox.dimensions[2]],
    #                                [0, 0, rd.uniform(1.,2.)], 
    #                                 rd.uniform(0.2,0.5)))

    for i, m in enumerate(meteorites):
        meteorites[i].update_position(dt)
        if m.pos[2] < 0:
            meteorites.pop(i)

    ## display
    #[m.display(scr, colors, view_angles, origin, scale) for m in meteorites]


    #---------------- obstacles ---------------------
    # obstacle1.display(scr, colors, view_angles, origin, scale)
    for meteorite in meteorites:
        meteorite.display(scr, colors, view_angles, origin, scale)

    #---------------- drone ---------------------  
    # u, x = solver.mpc(quad, quad.state, x_target, obstacle_list=[obstacle1], meteorites_list=[])
    # quad.update_state(u, model='non-linear')

    # populate obstacles
    # ACADO expects a fixed amount of obstacles, hence use a default set of obstacles out of range of the work area
    # and overwrite as needed
    obstacles = default_obstacle_set(n_obstacles, T)
    for i in range(len(meteorites)):
        o_start = i*4
        o_end = o_start+4
        obstacles[o_start:o_end] = np.array([
            np.ones(T) * meteorites[i].pos[0] + np.arange(0, T * constants.dt_solver, constants.dt_solver) *
            meteorites[i].vel[0],
            np.ones(T) * meteorites[i].pos[1] + np.arange(0, T * constants.dt_solver, constants.dt_solver) *
            meteorites[i].vel[1],
            np.ones(T) * meteorites[i].pos[2] + np.arange(0, T * constants.dt_solver, constants.dt_solver) *
            meteorites[i].vel[2],
            np.ones(T) * meteorites[i].size + constants.quadrotor_size
        ])

    x, u = acado.mpc(0, 1, np.array([np.array(np.hstack((quad.state, 0)))]), x, u, Y, yN, np.transpose(np.tile(Q, T)), Qf, 0, obstacles.T)
    # x, u = acado.mpc(0, 1, np.array([quad.state]), x, u, Y, yN, np.transpose(np.tile(Q, T)), Qf, 0, obstacles.T)

    inputs = u[0]
    if u[0][0] > quad.u_max[0] or u[0][0] < quad.u_min[0]:
        inputs = [0, 0, 0, 0]

    quad.update_state(inputs, model='non-linear')

    for i in range(T):
        p_x = projection([x[i, 0], x[i, 1], x[i, 2]], view_angles, origin, scale)
        pg.draw.circle(scr, colors.lime, p_x, 2)


    # display
    quad.display(scr, colors, view_angles, origin, scale, state=quad.state)

    if np.linalg.norm(quad.state-x_target) < 0.1:
        running = False
        print('ARRIVED :-D')


    # ---------- explosion -------------------
    #for i,m in enumerate(meteorites):
    #    if np.linalg.norm(drone1.pos-m.pos) < 1:
    #        meteorites.pop(i)
    #        colission = True
    #    localtime = display_explosion(scr, scale, colission, drone1.pos, dt, localtime, view_angles, origin)
    #    colission = False    


    # ---------- video -------------------
    filename = "frame" + str(loop_number) + ".jpg"
    pg.image.save(scr, "video_frames/"+filename)

    # ---------- update -------------------
    pg.display.flip()
    #print(loop_number)

    # ---------- exit -------------------
    for event in events:
        if event.type == pg.QUIT:
            pg.quit()
            pg.font.quit()
            running = False


video = False
if video:
    directory = 'video_frames'
    name = 'video_try69.mp4'
    size = screensize
    fps = 60
    make_video(directory, name, size, fps)







