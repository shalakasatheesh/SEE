#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import numpy as np
import matplotlib.pyplot as plt
import seaborn as sb
import os
import sys
import csv


def load_all_measurement_data():



    forward_measurements1=np.genfromtxt('../data/Other_team_data/Assignment_1_2/group_1_reading/front_measurement.csv',delimiter=',',skip_header=1)
    left_measurements1=np.genfromtxt('../data/Other_team_data/Assignment_1_2/group_1_reading/left_measurement.csv',delimiter=',',skip_header=1)
    right_measurements1=np.genfromtxt('../data/Other_team_data/Assignment_1_2/group_1_reading/right_measurement.csv',delimiter=',',skip_header=1)

    forward_measurements2=np.genfromtxt('../data/Other_team_data/Assignment_1_2/group_2_reading/Forward.csv',delimiter=',',skip_header=1)
    left_measurements2=np.genfromtxt('../data/Other_team_data/Assignment_1_2/group_2_reading/Left.csv',delimiter=',',skip_header=1)
    right_measurements2=np.genfromtxt('../data/Other_team_data/Assignment_1_2/group_2_reading/Right.csv',delimiter=',',skip_header=1)

    forward_measurements3=np.genfromtxt('../data/Other_team_data/Assignment_1_2/group_3_reading/straight_end_poses.csv',delimiter=',',skip_header=1)
    left_measurements3=np.genfromtxt('../data/Other_team_data/Assignment_1_2/group_3_reading/left_end_poses.csv',delimiter=',',skip_header=1)
    right_measurements3=np.genfromtxt('../data/Other_team_data/Assignment_1_2/group_3_reading/right_end_poses.csv',delimiter=',',skip_header=1)

    # forward_measurements4=np.genfromtxt('../data/Other_team_data/Assignment_1_2/group_4_reading/front_measurement.csv',delimiter=',',skip_header=1)
    # left_measurements4=np.genfromtxt('../data/Other_team_data/Assignment_1_2/group_4_reading/left_measurement.csv',delimiter=',',skip_header=1)
    # right_measurements4=np.genfromtxt('../data/Other_team_data/Assignment_1_2/group_4_reading/right_measurement.csv',delimiter=',',skip_header=1)
   
    forward_measurements4=np.array([[0,0,0],[0,0,0]])
    left_measurements4=np.array([[0,0,0],[0,0,0]])
    right_measurements4=np.array([[0,0,0],[0,0,0]])



    return forward_measurements1,left_measurements1,right_measurements1,forward_measurements2,left_measurements2,right_measurements2,forward_measurements3,left_measurements3,right_measurements3,forward_measurements4,left_measurements4,right_measurements4
    


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
   
    
    
def load_log_data(motion_type):
    '''
    Import the log from EV3 data logs 
    
    
    '''
    if motion_type=='f':
        directory='../data/forward_logs'

    if motion_type=='l':
        directory='../data/left_logs'

    if motion_type=='r':
        directory='../data/right_logs'
    

    filename_counter=1
    forward_logs=np.asarray([])
    forward_dic={}

    for filename in os.listdir(directory):
        with open(os.path.join(directory,filename), 'r') as f: # open in read only mode
            current_filename='get_'+str(filename_counter)
            
            
            if (filename_counter<8):
                #np.append(forward_logs,np.genfromtxt(f,delimiter=',',skip_header=1),axis=0)
                forward_dic[current_filename]=np.genfromtxt(f,delimiter=',',skip_header=1)


            filename_counter+=1

                
                
           


    return forward_dic


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

        ## x2,y2 are the values of front marker
        ## Bringing the measurements to the global coordinate system by accounting for offset

        x2=(measurement[0]+offset_pos[0])
        y2=(measurement[1]+offset_pos[1])

       


        ## x1,y1 are the values of rear marker
        ## Bringing the measurements to the global coordinate system by accounting for offset
        
        x1=(measurement[2]+offset_pos[0])
        y1=(measurement[3]+offset_pos[1])   

      
        
        
        
       # Final postions wrt to global coordinates

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
    Print data to txt file


    '''
    
    try:


        f=open(file_path+file_name,'x') ##Open/Create a file to store data
        for data_point in data_to_file:
            f.write(str(data_point)+'\n')

        f.close   


    except :

        print("\n File already exist in results folder \n")

def plot_log_data(log_forward,log_left,log_right):

    figure2=plt.figure(figsize=(6,12))
    ax2=figure2.add_subplot(111)
    

    ax2.scatter(log_forward['t1'][:,0],log_forward['t1'][:,1],color='tab:blue',s=1,label='Trial_1-Forward')
    ax2.scatter(log_forward['t2'][:,0],log_forward['t2'][:,1],color='tab:orange',s=1,label='Trial_2-Forward')
    ax2.scatter(log_forward['t3'][:,0],log_forward['t3'][:,1],color='tab:green',s=1,label='Trial_3-Forward')


    ax2.scatter(log_left['t1'][:,0],log_left['t1'][:,1],color='tab:red',s=1,label='Trial_1-left')
    ax2.scatter(log_left['t2'][:,0],log_left['t2'][:,1],color='tab:purple',s=1,label='Trial_2-left')
    ax2.scatter(log_left['t3'][:,0],log_left['t3'][:,1],color='tab:brown',s=1,label='Trial_3-left')


    ax2.scatter(log_right['t1'][:,0],log_right['t1'][:,1],color='tab:pink',s=1,label='Trial_1-right')   
    ax2.scatter(log_right['t2'][:,0],log_right['t2'][:,1],color='tab:gray',s=1,label='Trial_2-right')
    ax2.scatter(log_right['t3'][:,0],log_right['t3'][:,1],color='tab:olive',s=1,label='Trial_3-right')

    ax2.scatter(0,0,label='Start')


    keys=['t1','t2','t3']

    for key in keys:
        last_pos=[log_forward[key][-1,0],log_forward[key][-1,1]]
        last_dir=[np.cos(log_forward[key][-1,2]),np.sin(log_forward[key][-1,2])]
        ax2.quiver(last_pos[0],last_pos[1],last_dir[0],last_dir[1])

    for key in keys:
        last_pos=[log_left[key][-1,0],log_left[key][-1,1]]
        last_dir=[np.cos(log_left[key][-1,2]),np.sin(log_left[key][-1,2])]
        ax2.quiver(last_pos[0],last_pos[1],last_dir[0],last_dir[1])

    for key in keys:
        last_pos=[log_right[key][-1,0],log_right[key][-1,1]]
        last_dir=[np.cos(log_right[key][-1,2]),np.sin(log_right[key][-1,2])]
        ax2.quiver(last_pos[0],last_pos[1],last_dir[0],last_dir[1])

    ax2.quiver(0,0,0,1)

    
    ax2.set(title="Robot Path from logged data",xlabel="X(cm)",ylabel="Y(cm)")
    plt.legend()
    plt.grid()


    plt.show()

   

def plot_all_measurements(pose_forward,pose_left,pose_right,pose_forward1,pose_left1,pose_right1,pose_forward2,pose_left2,pose_right2,pose_forward3,pose_left3,pose_right3,pose_forward4,pose_left4,pose_right4):
    
    '''
    Init pos is alwasys [0,0,0]
    
    '''

    ##Define main graph parameters
    width=0.002




    
    figure1=plt.figure(figsize=(6,12))
    ax1=figure1.add_subplot(111)


    ## Forward motion

   #Fiding the direction vectors u & v from the angles
    u_f=-1.0*np.sin(np.deg2rad(pose_forward[:,2]))
    v_f=1.0*np.cos(np.deg2rad(pose_forward[:,2]))

    u_f1=-1.0*np.sin((pose_forward1[:,2]))
    v_f1=1.0*np.cos((pose_forward1[:,2]))

    u_f2=-1.0*np.sin(np.deg2rad(pose_forward2[:,2]))
    v_f2=1.0*np.cos(np.deg2rad(pose_forward2[:,2]))

    u_f3=-1.0*np.sin((pose_forward3[:,2]))
    v_f3=1.0*np.cos((pose_forward3[:,2]))

    u_f4=-1.0*np.sin(np.deg2rad(pose_forward4[:,2]))
    v_f4=1.0*np.cos(np.deg2rad(pose_forward4[:,2]))
    


    ## Left motion

   #Fiding the direction vectors u & v from the angles
    u_l=-1*np.sin(np.deg2rad(pose_left[:,2]))
    v_l=1*np.cos(np.deg2rad(pose_left[:,2]))

    u_l1=-1*np.sin((pose_left1[:,2]))
    v_l1=1*np.cos((pose_left1[:,2]))

    u_l2=-1*np.sin(np.deg2rad(pose_left1[:,2]))
    v_l2=1*np.cos(np.deg2rad(pose_left1[:,2]))

    u_l3=-1*np.sin((pose_left1[:,2]))
    v_l3=1*np.cos((pose_left1[:,2]))

    u_l4=-1*np.sin(np.deg2rad(pose_left1[:,2]))
    v_l4=1*np.cos(np.deg2rad(pose_left1[:,2]))




    ## Right motion

   #Fiding the direction vectors u & v from the angles
    u_r=-1.0*np.sin(np.deg2rad(pose_right[:,2]))
    v_r=1.0*np.cos(np.deg2rad(pose_right[:,2]))

    u_r1=-1.0*np.sin((pose_right1[:,2]))
    v_r1=1.0*np.cos((pose_right1[:,2]))

    u_r2=-1.0*np.sin(np.deg2rad(pose_right2[:,2]))
    v_r2=1.0*np.cos(np.deg2rad(pose_right2[:,2]))

    u_r3=-1.0*np.sin((pose_right3[:,2]))
    v_r3=1.0*np.cos((pose_right3[:,2]))



    ##Plotting
    
    ax1.quiver(pose_forward[:,0],pose_forward[:,1],u_f,v_f,color='r',width=width,linewidths=0.1,minshaft=1,label='Forward Group 4(ours)')
    ax1.quiver(pose_forward1[:,0],pose_forward1[:,1],u_f1,v_f1,color='b',width=width,linewidths=0.1,minshaft=1,label='Forward Group1')


    ax1.quiver(pose_left[:,0],pose_left[:,1],u_l,v_l,color='b',width=width,linewidths=0.1,label='Left Group 4(Ours)')
    ax1.quiver(pose_left1[:,0],pose_left1[:,1],u_l1,v_l1,color='b',width=width,linewidths=0.1,label='Left Group 1')


    ax1.quiver(pose_right[:,0],pose_right[:,1],u_r,v_r,color='g',width=width,linewidths=0.1,label='Right Group 4(Ours)')
    ax1.quiver(pose_right1[:,0],pose_right1[:,1],u_r1,v_r1,color='g',width=width,linewidths=0.1,label='Right Group 1')




    ax1.quiver(0,0,0,1,label='Start')

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
      
def plot_measurement(pose_forward,pose_left,pose_right):
    '''
    Init pos is alwasys [0,0,0]
    
    '''

    ##Define main graph parameters
    width=0.002




    
    figure1=plt.figure(figsize=(4,8))
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
    ax1.quiver(0,0,0,1,label='Start')

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

def plot_n_measurement(pose_forward,pose_left,pose_right,log_forward,log_left,log_right):

    width=0.009
    keys=['t1','t2','t3']


    figure1=plt.figure(figsize=(6,12))
    ax1=figure1.add_subplot(111)


    #Fiding the direction vectors u & v from the angles
    u_f=-1.0*np.sin(np.deg2rad(pose_forward[0:3,2]))
    v_f=1.0*np.cos(np.deg2rad(pose_forward[0:3,2]))

    ax1.quiver(pose_forward[0,0],pose_forward[0,1],u_f[0],v_f[0],color='b',width=width,linewidths=0.1,label='Trial 1 Measurements ')
    ax1.quiver(pose_forward[1,0],pose_forward[1,1],u_f[1],v_f[1],color='g',width=width,linewidths=0.1,label='Trial 2 Measurements ')
    ax1.quiver(pose_forward[2,0],pose_forward[2,1],u_f[2],v_f[2],color='r',width=width,linewidths=0.1,label='Trial 3 Measurements ')
    ax1.scatter(log_forward['t1'][:,0],log_forward['t1'][:,1],color='tab:blue',s=1,label='Trial 1 Logged')
    ax1.scatter(log_forward['t2'][:,0],log_forward['t2'][:,1],color='tab:orange',s=1,label='Trial 2 Logged')
    ax1.scatter(log_forward['t3'][:,0],log_forward['t3'][:,1],color='tab:green',s=1,label='Trial 3 Logged')
    ax1.quiver(0,0,0,1,label='Start')
    for key in keys:
        last_pos=[log_forward[key][-1,0],log_forward[key][-1,1]]
        last_dir=[np.cos(log_forward[key][-1,2]),np.sin(log_forward[key][-1,2])]
        ax1.quiver(last_pos[0],last_pos[1],last_dir[0],last_dir[1])

    ax1.set(title="Measured Movements vs Log Data of Forward Motion ",xlabel="X(cm)",ylabel="Y(cm)")


    figure2=plt.figure(figsize=(6,12))
    ax2=figure2.add_subplot(111)

   




    ##Left

    #Fiding the direction vectors u & v from the angles
    u_l=-1*np.sin(np.deg2rad(pose_left[0:3,2]))
    v_l=1*np.cos(np.deg2rad(pose_left[0:3,2]))

    ax2.quiver(pose_left[0,0],pose_left[0,1],u_l[0],v_l[0],color='b',width=width,linewidths=0.1,label='Trial 1 Measurements ')
    ax2.quiver(pose_left[1,0],pose_left[1,1],u_l[1],v_l[1],color='g',width=width,linewidths=0.1,label='Trial 2 Measurements ')
    ax2.quiver(pose_left[2,0],pose_left[2,1],u_l[2],v_l[2],color='r',width=width,linewidths=0.1,label='Trial 3 Measurements ')
    ax2.scatter(log_left['t1'][:,0],log_left['t1'][:,1],color='tab:red',s=1,label='Trial 1 logged')
    ax2.scatter(log_left['t2'][:,0],log_left['t2'][:,1],color='tab:purple',s=1,label='Trial 2 logged')
    ax2.scatter(log_left['t3'][:,0],log_left['t3'][:,1],color='tab:brown',s=1,label='Trial 3 logged')
    
    for key in keys:
        last_pos=[log_left[key][-1,0],log_left[key][-1,1]]
        last_dir=[np.cos(log_left[key][-1,2]),np.sin(log_left[key][-1,2])]
        ax2.quiver(last_pos[0],last_pos[1],last_dir[0],last_dir[1])

    ax2.quiver(0,0,0,1,label='Start')


    ax2.set(title="Measured Movements vs Log Data of Left Motion ",xlabel="X(cm)",ylabel="Y(cm)")

   




    ##Right
    #Fiding the direction vectors u & v from the angles


    figure3=plt.figure(figsize=(6,12))
    ax3=figure3.add_subplot(111)

    u_r=-1.0*np.sin(np.deg2rad(pose_right[0:3,2]))
    v_r=1.0*np.cos(np.deg2rad(pose_right[0:3,2]))
    ax3.quiver(pose_right[0,0],pose_right[0,1],u_r[0],v_r[0],color='b',width=width,linewidths=0.1,label='Trial 1 Measurements ')
    ax3.quiver(pose_right[1,0],pose_right[1,1],u_r[1],v_r[1],color='g',width=width,linewidths=0.1,label='Trial 2 Measurements ')
    ax3.quiver(pose_right[2,0],pose_right[2,1],u_r[2],v_r[2],color='r',width=width,linewidths=0.1,label='Trial 3 Measurements ')
    ax3.scatter(log_right['t1'][:,0],log_right['t1'][:,1],color='tab:pink',s=1,label='Trial 1 logged')   
    ax3.scatter(log_right['t2'][:,0],log_right['t2'][:,1],color='tab:gray',s=1,label='Trial 2 logged')
    ax3.scatter(log_right['t3'][:,0],log_right['t3'][:,1],color='tab:olive',s=1,label='Trial 3 logged')

    for key in keys:
        last_pos=[log_right[key][-1,0],log_right[key][-1,1]]
        last_dir=[np.cos(log_right[key][-1,2]),np.sin(log_right[key][-1,2])]
        ax3.quiver(last_pos[0],last_pos[1],last_dir[0],last_dir[1])






    ax3.quiver(0,0,0,1,label='Start')
    ax3.set(title="Measured Movements vs Log Data of Right Motion ",xlabel="X(cm)",ylabel="Y(cm)")

    







    plt.legend()
    plt.grid()
    plt.show()


if __name__=='__main__':



    ##Load the measurement data
    forward_mesurements,left_mesurements,right_mesurements=load_measurement_data()

   




    #Transform measurements to degrees 
    measured_pose_forwards=transform_mesurement_2_pose(forward_mesurements)
    measured_pose_left    =transform_mesurement_2_pose(left_mesurements)
    measured_pose_right   = transform_mesurement_2_pose(right_mesurements)
   
     
    plot_measurement(measured_pose_forwards,measured_pose_left,measured_pose_right)

    #Load log data from EV3
    forward_log=load_log_data('f')
    left_log=load_log_data('l')
    right_log=load_log_data('r')
    
    ##Load first three into a new 
    forward_t3={}
    forward_t3['t1']=forward_log['get_1']
    forward_t3['t2']=forward_log['get_4']
    forward_t3['t3']=forward_log['get_5']


    left_t3={}
    left_t3['t1']=left_log['get_2']
    left_t3['t2']=left_log['get_3']
    left_t3['t3']=left_log['get_4']


    right_t3={}
    right_t3['t1']=right_log['get_1']
    right_t3['t2']=right_log['get_2']
    right_t3['t3']=right_log['get_7']

    # plot_log_data(forward_t3,left_t3,right_t3)

    # plot_n_measurement(measured_pose_forwards,measured_pose_left,measured_pose_right,forward_t3,left_t3,right_t3)

  
    forward_mesurements1,left_mesurements1,right_mesurements1,\
    forward_mesurements2,left_mesurements2,right_mesurements2,\
        forward_mesurements3,left_mesurements3,right_mesurements3,\
            forward_mesurements4,left_mesurements4,right_mesurements4\
    =load_all_measurement_data()



    plot_all_measurements(measured_pose_forwards,measured_pose_left,measured_pose_right,forward_mesurements1,left_mesurements1,right_mesurements1,\
        forward_mesurements2,left_mesurements2,right_mesurements2,\
            forward_mesurements3,left_mesurements3,right_mesurements3,\
                forward_mesurements4,left_mesurements4,right_mesurements4)


  
    








    ###############################
    ## Print to files
    ##############################

    # print_data_to_txt('measured_forward_movement',measured_pose_forwards)
    # print_data_to_txt('measured_left_movement',measured_pose_left)
    # print_data_to_txt('measured_right_movement',measured_pose_right)


    
    # load_log_data()