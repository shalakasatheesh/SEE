#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
from scipy.stats import norm,iqr
import matplotlib.pyplot as plt



def transform_mesurement_2_pose(measurements,initial_pos=np.array([0.0,69.0,0.0,-168.0]),offset_pos=np.array([-56,-67])):
    '''
    Transform the raw measurement data to the global coordinate sytem

    This function is specific to group 4 data
    
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

        ##Transform the angle measurement starting fromm the y axis instead of x and anticlockwise positive
        theta=np.round(np.rad2deg(np.arctan2(y2-y1,x2-x1))-90.0,2)
        
       
       #Dividing by 10 to convert to cm
        pose[index,0]=x/10
        pose[index,1]=y/10
        pose[index,2]=theta
    

    return pose


def load_data():
    '''
    Import all measurement Data 


    Returns
    -----------
    group_4_data:   The [[forward],[left],[right]] motions of our group in order [x,y,orientation]
    group_all_data: The [[forward],[left],[right]] motions of all other group in order [x,y,orientation]

    
    
    '''

    ##First import our own data (Group 4)

    raw_forward_measurements4=np.genfromtxt('../data/Assignment_1_2/measurements/pain_forward.csv',delimiter=',',skip_header=1)
    raw_left_measurements4=np.genfromtxt('../data/Assignment_1_2/measurements/pain_left.csv',delimiter=',',skip_header=1)
    raw_right_measurements4=np.genfromtxt('../data/Assignment_1_2/measurements/pain_right.csv',delimiter=',',skip_header=1)



    #Transform measurements to degrees 
    forward_measurements4=transform_mesurement_2_pose(raw_forward_measurements4)
    left_measurements4    =transform_mesurement_2_pose(raw_left_measurements4)
    right_measurements4   = transform_mesurement_2_pose(raw_right_measurements4)


    ##All other team data

    
    forward_measurements1=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_1_reading/front_measurement.csv',delimiter=',',skip_header=1)
    left_measurements1=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_1_reading/left_measurement.csv',delimiter=',',skip_header=1)
    right_measurements1=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_1_reading/right_measurement.csv',delimiter=',',skip_header=1)

    ## Team 2 have bad data . Will not use it

    forward_measurements2=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_2_reading/Forward.csv',delimiter=',',skip_header=1)
    left_measurements2=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_2_reading/Left.csv',delimiter=',',skip_header=1)
    right_measurements2=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_2_reading/Right.csv',delimiter=',',skip_header=1)


    ##Convert Team 3 angles staring from y axis and anti-clockwise postive

    forward_measurements3=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_3_reading/straight_end_poses.csv',delimiter=',',skip_header=1)
    forward_measurements3=forward_measurements3[:,1:]
    left_measurements3=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_3_reading/left_end_poses.csv',delimiter=',',skip_header=1)
    left_measurements3=left_measurements3[:,1:]
    right_measurements3=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_3_reading/right_end_poses.csv',delimiter=',',skip_header=1)
    right_measurements3=right_measurements3[:,1:]


    



    ##Team 5 has a different format in csv therefore have to filter it and had to convert angles to starting from y axis and anticlockwise positive

    all_measurements5=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_5_reading/Manual_measurement_group5.csv',delimiter=',',skip_header=1)
    forward_measurements5=all_measurements5[1:22,5:8]
    left_measurements5=all_measurements5[1:22,1:4]
    right_measurements5=all_measurements5[1:22,9:14]


    for index in range(len(forward_measurements3)):
        forward_measurements3[index,2]=-1*forward_measurements3[index,2]
        left_measurements3[index,2]=-1*left_measurements3[index,2]
        right_measurements3[index,2]=-1*right_measurements3[index,2]

        forward_measurements5[index,2]=-1*forward_measurements5[index,2]
        left_measurements5[index,2]=-1*left_measurements5[index,2]
        right_measurements5[index,2]=-1*right_measurements5[index,2]



    
    ## Team 6 has distances in cm and angles in rads therefore have to convert it 
    
    forward_measurements6=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_6_reading/straight_readings.csv',delimiter=' ',skip_header=1)
    left_measurements6=   np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_6_reading/left_readings.csv',    delimiter='',skip_header=1)
    right_measurements6=  np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_6_reading/right_readings.csv',   delimiter='',skip_header=1)
    
    for index in range(len(forward_measurements6)):
        forward_measurements6[index,2]=np.rad2deg(forward_measurements6[index,2])-90.0
        left_measurements6[index,2]=np.rad2deg(left_measurements6[index,2])-90.0
        right_measurements6[index,2]=np.rad2deg(right_measurements6[index,2])-90.0


    ##Combine all group data

    
    forward_measurements_all=np.concatenate((forward_measurements1,forward_measurements3,forward_measurements5,forward_measurements6),axis=0)

    
    left_measurements_all=np.concatenate((left_measurements1,left_measurements3,left_measurements5,left_measurements6),axis=0)
    
    
    right_measurements_all=np.concatenate((right_measurements1,right_measurements3,right_measurements5,right_measurements6),axis=0)


   

    
    group_4_data=np.hstack((forward_measurements4,left_measurements4,right_measurements4))
    group_all_data=np.hstack((forward_measurements_all,left_measurements_all,right_measurements_all))
    

        

    
    
    
    return group_4_data,group_all_data



def remove_outliers(data, pp1, pp2):
    '''
        Based on "Data Outlier Detection using the Chebyshev Theorem",
        Brett G. Amidan, Thomas A. Ferryman, and Scott K. Cooley

        Based on the lecture slide of SEE at HBRS


        Parameters
        -----------
        data -- A numpy array of discrete or continuous data
        pp1 -- likelihood of expected outliers (e.g. 0.1, 0.05 , 0.01)
        pp2 -- final likelihood of real outliers (e.g. 0.01, 0.001 , 0.0001)

        Returns
        -----------
        final_data: Data with outliers removed
    '''

    mu1 = np.mean(data)
    sigma1 = np.std(data)
    k = 1./ np.sqrt(pp1)
    odv1u = mu1 + k * sigma1
    odv1l = mu1 - k * sigma1


    new_data = data[np.where(data <= odv1u)[0]]
    new_data = new_data[np.where(new_data >= odv1l)[0]]
    mu2 = np.mean(new_data)
    sigma2 = np.std(new_data)
    k = 1./ np.sqrt(pp2)
    odv2u = mu2 + k * sigma2
    odv2l = mu2 - k * sigma2
    final_data = new_data[np.where(new_data <= odv2u)[0]]
    final_data = new_data[np.where(final_data >= odv2l)[0]]


    return final_data
    

def gaussian_fit(data):
    '''
    Take in a data set and fit to gaussian


    Parameters
    ----------
    data : has the form [[forward],[left],[right]]
    

    
    '''

    
    for motion_index in [[0,3],[3,6],[6,9]]:
        current_data=data[:,motion_index[0]:motion_index[1]]
        x_mean=np.mean(current_data[:,0])
        x_std =np.std(current_data[:,0])
        ## Freedman–Diaconis rule for calculating number of bins in a histogram
        ## https://en.wikipedia.org/wiki/Freedman%E2%80%93Diaconis_rule
        x_n_bins=int(round((np.max(current_data[:,0])-np.min(current_data[:,0]))/(2*iqr(current_data[:,0])*len(current_data)**(-1/3))))
        x_range=np.arange(np.min(current_data[:,0]),np.max(current_data[:,0]),0.1)
            




        y_mean=np.mean(current_data[:,1])
        y_std =np.std(current_data[:,1])
        y_n_bins=int(round((np.max(current_data[:,1])-np.min(current_data[:,1]))/(2*iqr(current_data[:,1])*len(current_data)**(-1/3))))
        y_range=np.arange(np.min(current_data[:,1]),np.max(current_data[:,1]),0.1)


        if motion_index[0]==0:
            plot_title_x="Forward data for x"
            plot_title_y="Forward data for y"

        if motion_index[0]==3:
            plot_title_x="Forward data for x"
            plot_title_y="Forward data for y"

        if motion_index[0]==6:
            plot_title_x="Forward data for x"
            plot_title_y="Forward data for y"

        plt.figure()
        plt.plot(x_range,norm.pdf(x_range,x_mean,x_std),label='Gaussian')
        plt.hist(current_data[:,0],bins=x_n_bins,ec='black',density=True,label='Histogram')
        plt.title(plot_title_x)
        plt.xlabel('Data')
        plt.ylabel('P(x)')
        plt.legend()
        plt.grid()


        plt.figure()

        
        plt.plot(y_range,norm.pdf(y_range,y_mean,y_std),label='Gaussian')
        plt.hist(current_data[:,1],bins=y_n_bins,ec='black',density=True,label='Histogram')
        plt.title(plot_title_y)
        plt.xlabel('Data')
        plt.ylabel('P(y)')
        plt.legend()
        plt.grid()




        plt.show()

   

    




if __name__=='__main__':
    group_4_data,group_all_data=load_data()
    gaussian_fit(group_all_data)