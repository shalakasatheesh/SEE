#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import numpy as np
import matplotlib.pyplot as plt
import seaborn as sb
import os
import sys
import csv


def load_measurement_data():
    '''
    Import data from the measurement files

    The measurement data was stored in the format given below
    [x_front,y_front,x_rear,y_rear]
    
    '''



    forward_measurements=np.genfromtxt('../data/measurements/pain_forward.csv',delimiter=',',skip_header=1)
    left_measurements=np.genfromtxt('../data/measurements/pain_forward.csv',delimiter=',',skip_header=1)
    right_measurements=np.genfromtxt('../data/measurements/pain_forward.csv',delimiter=',',skip_header=1)
    



    return forward_measurements,left_measurements,right_measurements
   
    
    
def load_log_data():
    '''
    Import the log from EV3
    
    
    '''
    filename_counter=1
    forward_logs=np.asarray([])

    for filename in os.listdir('../data/forward_logs'):
        with open(os.path.join('../data/forward_logs', filename), 'r') as f: # open in read only mode
            current_filename='get_'+str(filename_counter)
            filename_counter+=1
            # print(current_filename)
            # print(f)
            if (filename_counter%2)==0:
                # np.append(forward_logs,np.genfromtxt(f,delimiter=',',skip_header=1),axis=0)
                # print(filename[-20:-4])
                a=10
            if filename_counter==3:
                # print(filename)
                aa=np.genfromtxt(f,delimiter=',',skip_header=1)
                # print(aa)
                # print(aa.shape)


def transform_mesurement_2_pose(measurements,initial_pos=np.array([0.0,69.0,0.0,-168.0])):
    '''
    Transform the raw measurement data to the global coordinate sytem
    
    The measurement data was stored in the format given below
    [x_front,y_front,x_rear,y_rear]
    All measurements are in mm
    For the pose it will be convered to cm

    Parameter
    ---------------
    measurements : The data array with manual measurements
    initial_pos  : The starting position of front and rear marker

    Returns
    -------------
    pose         : The [x,y,theta] of the robot ->np.array
    
    '''
    pose=np.zeros((measurements.shape[0],3),dtype=float)
    

    



    for index,measurement in enumerate(measurements):
        
        x= ((measurement[2]-initial_pos[2])+(measurement[0]-initial_pos[0]))/2
        y= ((measurement[3]-initial_pos[3])+(measurement[1]-initial_pos[1]))/2

        y1=(measurement[3]+initial_pos[3])
        y2=(measurement[1]-initial_pos[1])
        x1=(measurement[2]+initial_pos[2])
        x2=(measurement[0]-initial_pos[0])
        # np.printoptions(suppress=True)

        ##The 90 degrees is minused to transform the anngle measiremetn startig fromm the y axis instead of x 
        theta=np.round(np.rad2deg(np.arctan2(y2-y1,x2-x1))-90.0,4)
        # theta='{:.5f}'.format(theta)
       
       #Dividing by 10 to convert to cm
        pose[index,0]=x/10
        pose[index,1]=y/10
        pose[index,2]=theta
    

    return pose


def print_data_to_csv(file_name,data_to_file,file_path='../data/'):

    '''


    '''
    
    try:


        f=open(file_path+file_name,'x') ##Open/Create a file to store data
        for data_point in data_to_file:
            f.write(str(data_point)+'\n')

        f.close   


    except :

        print("\n File already exist in results folder \n")

    

    # with open('employee_file.csv', mode='w') as employee_file:
    #     employee_writer = csv.writer(employee_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    #     employee_writer.writerow(['John Smith', 'Accounting', 'November'])
    #     employee_writer.writerow(['Erica Meyers', 'IT', 'March'])



       
def plot_measurement(pose):
    '''
    Init pos is alwasys [0,0,0]
    
    '''
    # plt.scatter(pose[:,0],pose[:,1],s=5)
    u=-1.0*np.sin(np.deg2rad(pose[:,2]))
    v=1.0*np.cos(np.deg2rad(pose[:,2]))
    print(u)
    plt.quiver(pose[:,0],pose[:,1],u,v,color='r',width=0.005)
    x_max=np.max(pose[:,0])
    x_min=np.min(pose[:,0])

    y_max=np.max(pose[:,1])
    y_min=np.min(pose[:,1])
    plt.xlim([x_min-30,x_max+30])
    plt.ylim([-5,y_max+30])
    plt.show()


if __name__=='__main__':



    ##Load the measurement data
    forward_mesurements,left_mesurements,right_mesurements=load_measurement_data()

    #Trans
    measured_pose_forwards=transform_mesurement_2_pose(forward_mesurements)
    measured_pose_left    =transform_mesurement_2_pose(left_mesurements)
    measured_pose_right   = transform_mesurement_2_pose(right_mesurements)
    print(measured_pose_left)
    # plt.scatter(measured_pose_forwards[:,0],measured_pose_forwards[:,1])
    # plt.show()
    plot_measurement(measured_pose_left)

    # print_data_to_csv('measured_left_n2cle',measured_pose_left)


    
    # load_log_data()