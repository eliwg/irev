"""
University of Michigan
ENG100-400
"""

import sys
sys.path.append('C:\\AirSim\\PythonClient')
sys.path.append('C:\\AirSim\\PythonClient\\multirotor')

############### import a few useful libraries ###########

import setup_path
import time
import numpy as np
import math
import matplotlib.pyplot as plt

############### establish the link to AirSim ###########

import airsim              # import AirSim API
import E100_functions      # import drone simulator library

dt = E100_functions.dt()  
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

#### Copy and paste the above in your own flight controller #####

alpha_alt = 0.7
alpha_lidar = 0.2

target_alt = 8
K_P = 1
K_I = 0.
K_D = 2
alt_integration_term = 0
alt_flag = 1
throttle = 0.5  # initialize Throttle
##############################################
target_front_dist = 8
pitch_K_P = 0.5
pitch_K_I = 0.0
pitch_K_D = 0.75
pitch_integration_term = 0
pitch_flag = 1
desired_pitch = 0
##############################################
target_right_dist = 5
roll_K_P = 0.5
roll_K_I = 0.001
roll_K_D = 0.65
roll_integration_term = 0
roll_flag = 1
desired_roll = 0 
desired_yaw = 0

start = time.time()

altitude_sensor_flag = 1
Lidar_sensor_flag = 1

x_pos = []
y_pos = []
direction = 'f';
wall = 'r';
while True:
           
    now = time.time()
    if now - start > 122:
        target_alt = 0
    if now - start > 125:   
        break
    
    x,y = E100_functions.get_XY(client)
    x_pos.append(x)
    y_pos.append(y)
   
    #################### get sensor readings #####################
    if altitude_sensor_flag==1:
        altitude_n1=0
        altitude_sensor_flag=0
    if Lidar_sensor_flag == 1:
        front1, right1, left1, back1 = E100_functions.get_lidars(client)
               
    altitude_n0 = E100_functions.get_altitude(client) # read quadcopter's altitude 
    altitude = alpha_alt*altitude_n0 + (1-alpha_alt)*altitude_n1  # apply low-pass filter
    altitude_n1 = altitude
    
    roll, pitch, yaw = E100_functions.get_orientation(client) # read quadcopter's attitude
    
    front, right, left, back = E100_functions.get_lidars(client)    # read LIDAR readings
    
    
    if(now - start < 5):
        left = alpha_lidar*left + (1-alpha_lidar)*left1
        right = alpha_lidar*right + (1-alpha_lidar)*right1
        front = alpha_lidar*front + (1-alpha_lidar)*front1
        back = alpha_lidar*back + (1-alpha_lidar)*back1
    else:
    
        if(direction == 'f'):
            if(wall == 'r'):
                if(front < 6 and right > 6):
                    direction = 'r'
                    wall = 'b'
                elif(front < 6 and right < 6):
                    direction = 'l'
                    wall = 'f'
            else:
                if(front < 6 and left > 6):
                    direction = 'l'
                    wall = 'b'
                elif(front < 6 and left < 6):
                    direction = 'r'
                    wall = 'f'
        elif(direction == 'b'):
            if(wall == 'r'):
                if(back < 6 and right > 6):
                    direction = 'r'
                    wall = 'f'
                elif(back < 6 and right < 6):
                    direction = 'l'
                    wall = 'b'
            else:
                if(back < 6 and left > 6):
                    direction = 'l'
                    wall = 'f'
                elif(back < 6 and left < 6):
                    direction = 'r'
                    wall = 'b'
        elif(direction == 'r'):
            if(wall == 'f'):
                if(right < 6 and front > 6):
                    direction = 'f'
                    wall = 'l'
                elif(right < 6 and front < 6):
                    direction = 'b'
                    wall = 'r'
            else:
                if(right < 6 and back > 6):
                    direction = 'b'
                    wall = 'l'
                elif(right < 6 and back < 6):
                    direction = 'f'
                    wall = 'r'
        else:
            if(wall == 'f'):
                if(left < 6 and front > 6):
                    direction = 'f'
                    wall = 'r'
                elif(left < 6 and front < 6):
                    direction = 'b'
                    wall = 'l'
            else:
                if(left < 6 and back > 6):
                    direction = 'b'
                    wall = 'r'
                elif(left < 6 and back < 6):
                    direction = 'f'
                    wall = 'l'
                
    
        if(direction == 'r'):
            if(wall == 'u'):
                back = alpha_lidar*back + (1-alpha_lidar)*4
                front = alpha_lidar*front + (1-alpha_lidar)*5
            else:
                back = alpha_lidar*back + (1-alpha_lidar)*5
                front = alpha_lidar*front + (1-alpha_lidar)*4
            left = alpha_lidar*left + (1-alpha_lidar)*2
            right = alpha_lidar*right + (1-alpha_lidar)*right1
        elif(direction == 'l'):
            if(wall == 'u'):
                back = alpha_lidar*back + (1-alpha_lidar)*4
                front = alpha_lidar*front + (1-alpha_lidar)*5
            else:
                back = alpha_lidar*back + (1-alpha_lidar)*5
                front = alpha_lidar*front + (1-alpha_lidar)*4
            right = alpha_lidar*right + (1-alpha_lidar)*2
            left = alpha_lidar*left + (1-alpha_lidar)*left1
        elif(direction == 'b'):
            if(wall == 'r'):
                left = alpha_lidar*left + (1-alpha_lidar)*4
                right = alpha_lidar*right + (1-alpha_lidar)*5
            else:
                left = alpha_lidar*left + (1-alpha_lidar)*5
                right = alpha_lidar*right + (1-alpha_lidar)*4
            front = alpha_lidar*front + (1-alpha_lidar)*2
            back = alpha_lidar*back + (1-alpha_lidar)*back1
        else:
            if(wall == 'r'):
                left = alpha_lidar*left + (1-alpha_lidar)*4
                right = alpha_lidar*right + (1-alpha_lidar)*5
            else:
                left = alpha_lidar*left + (1-alpha_lidar)*5
                right = alpha_lidar*right + (1-alpha_lidar)*4
            back = alpha_lidar*back + (1-alpha_lidar)*2
            front = alpha_lidar*front + (1-alpha_lidar)*front1
        
    
    
    front1 = front
    right1 = right
    left1 = left
    
    if alt_flag == 1:
        error_old = target_alt-altitude
        alt_flag = 0
    else:
        error_old = error    
    error= target_alt-altitude
    alt_integration_term += error*dt
    alt_differential_term = (error - error_old)/dt  
    throttle = K_P*error + K_I*alt_integration_term + K_D*alt_differential_term
    if throttle < 0:
        throttle = 0
    elif throttle > 1:
        throttle = 1       
    
    ##############################################################
    #front dist hold part(PID controller+activation function)####
    ##############################################################     
    if pitch_flag == 1:
        pitch_error_old = target_front_dist-front
        pitch_flag = 0
    else:
        pitch_error_old = pitch_error    
    pitch_error= target_front_dist-front
    pitch_integration_term += pitch_error*dt
    pitch_differential_term = (pitch_error - pitch_error_old)/dt
    desired_pitch = pitch_K_P*pitch_error + pitch_K_I*pitch_integration_term + pitch_K_D*pitch_differential_term
    desired_pitch = math.degrees(0.15*np.tanh(desired_pitch))  # use tanh function to limit the maximum and minimum of the pitch value
    
    ##############################################################
    #center dist hold part(PID controller+activation function)####
    ##############################################################    
    if roll_flag == 1:
        roll_error_old = right - left
        roll_flag = 0
    else:
        roll_error_old = roll_error    
    roll_error=  right - left
    roll_integration_term += roll_error*dt
    roll_differential_term = (roll_error - roll_error_old)/dt    
    desired_roll = roll_K_P*roll_error + roll_K_I*roll_integration_term + roll_K_D*roll_differential_term
    desired_roll = math.degrees(0.2*np.tanh(0.5*desired_roll))
    desired_yaw = 0
    
    #control signal 
    E100_functions.set_quadcopter(client,desired_roll,desired_pitch,desired_yaw,throttle)
    


plt.scatter(x_pos,y_pos)

#### Copy and paste the following in your own flight controller #####    

############### release the link to AirSim ###########
client.armDisarm(False)
client.enableApiControl(False)






















