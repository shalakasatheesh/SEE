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
    left_measurements=np.genfromtxt('../data/measurements/pain_left.csv',delimiter=',',skip_header=1)
    right_measurements=np.genfromtxt('../data/measurements/pain_right.csv',delimiter=',',skip_header=1)
    



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


def transform_mesurement_2_pose(measurements,initial_pos=np.array([0.0,69.0,0.0,-168.0]),offset_pos=np.array([-56,-67])):
    '''
    Transform the raw measurement data to the global coordinate sytem
    
    The measurement data was stored in the format given below
    [x_front,y_front,x_rear,y_rear]==[x2,y2,x1,y1]
    All measurements are in mm
    For the pose it will be convered to cm

    Parameter
    ---------------
    measurements : The data array with manual measurements in the order [x2,y2,x1,y1]
    initial_pos  : The starting position of front and rear marker in that order
    offset_pos   : Offset of the origin of the marker used for measurements

    Returns
    -------------
    pose         : The [x,y,theta] of the robot ->np.array
    
    '''
    pose=np.zeros((measurements.shape[0],3),dtype=float)
    
    


    

    



    for index,measurement in enumerate(measurements):


        # x1=(measurement[2]+initial_pos[2])
        # y1=(measurement[3]+initial_pos[3])

        # x2=(measurement[0]-initial_pos[0])
        # y2=(measurement[1]-initial_pos[1])

        # x= ((measurement[2]-initial_pos[2])+(measurement[0]-initial_pos[0]))/2
        # y= ((measurement[3]-initial_pos[3])+(measurement[1]-initial_pos[1]))/2

        ## x2,y2 are the values of front marker
        ## Bringing the measurements to the global coordinate system by accounting for offset

        x2=(measurement[0]+offset_pos[0])
        y2=(measurement[1]+offset_pos[1])

       


        ## x1,y1 are the values of rear marker
        ## Bringing the measurements to the global coordinate system by accounting for offset
        
        x1=(measurement[2]+offset_pos[0])
        y1=(measurement[3]+offset_pos[1])

       

      
        
        
        
       # FInal postions wrt to global coordinates

        x= ((x1-initial_pos[2])+(x2-initial_pos[0]))/2
        y= ((y1-initial_pos[3])+(y2-initial_pos[1]))/2

    

        

       

        
        # np.printoptions(suppress=True)

        ##The 90 degrees is minused to transform the anngle measiremetn startig fromm the y axis instead of x 
        theta=np.round(np.rad2deg(np.arctan2(y2-y1,x2-x1))-90.0,4)
        
       
       #Dividing by 10 to convert to cm
        pose[index,0]=x/10
        pose[index,1]=y/10
        pose[index,2]=theta
    

    return pose


def print_data_to_txt(file_name,data_to_file,file_path='../data/'):

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



       
def plot_measurement(pose_forward,pose_left,pose_right):
    '''
    Init pos is alwasys [0,0,0]
    
    '''

    ##Define main graph parameters
    width=0.002




    
    figure1=plt.figure(figsize=(8,4))
    ax1=figure1.add_subplot(111)


    ## Forward motion

   #Fiding the direction vectors u & v from the angles
    u_f=-1.0*np.sin(np.deg2rad(pose_forward[:,2]))
    v_f=1.0*np.cos(np.deg2rad(pose_forward[:,2]))


    ## Left motion

   #Fiding the direction vectors u & v from the angles
    u_l=-1*np.sin(np.deg2rad(pose_left[:,2]))
    v_l=1*np.cos(np.deg2rad(pose_left[:,2]))


    ## Right motion

   #Fiding the direction vectors u & v from the angles
    u_r=-1.0*np.sin(np.deg2rad(pose_right[:,2]))
    v_r=1.0*np.cos(np.deg2rad(pose_right[:,2]))
    
    ax1.quiver(pose_forward[:,0],pose_forward[:,1],u_f,v_f,color='r',width=width,linewidths=0.1,minshaft=1,label='Forward')
    ax1.quiver(pose_left[:,0],pose_left[:,1],u_l,v_l,color='b',width=width,linewidths=0.1,label='Left')
    ax1.quiver(pose_right[:,0],pose_right[:,1],u_r,v_r,color='g',width=width,linewidths=0.1,label='Right')

    x_max=np.max(pose_forward[:,0])
    x_min=np.min(pose_forward[:,0])

    y_max=np.max(pose_forward[:,1])
    y_min=np.min(pose_forward[:,1])
    plt.xlim([x_min-30,x_max+30])
    plt.ylim([-5,y_max+30])
    ax1.set(title="Measured Movements",xlabel="X(cm)",ylabel="Y(cm)")
    plt.legend()
    plt.grid()
    plt.show()


if __name__=='__main__':



    ##Load the measurement data
    forward_mesurements,left_mesurements,right_mesurements=load_measurement_data()

    ##Load log data from EV3
    # forward_log=load_log_data()

    #Transform measurements to degrees 
    measured_pose_forwards=transform_mesurement_2_pose(forward_mesurements)
    measured_pose_left    =transform_mesurement_2_pose(left_mesurements)
    measured_pose_right   = transform_mesurement_2_pose(right_mesurements)
   
    # print(f'left pose \n {measured_pose_left}')
    # print(f'right pose \n {measured_pose_right}')
    # print(f'straght pose \n {measured_pose_forwards}')
   
    plot_measurement(measured_pose_forwards,measured_pose_left,measured_pose_right)
    








    ###############################
    ## Print to files
    ##############################

    print_data_to_txt('measured_forward_movement',measured_pose_forwards)
    print_data_to_txt('measured_left_movement',measured_pose_left)
    print_data_to_txt('measured_right_movement',measured_pose_right)


    
    # load_log_data()