#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
from scipy.stats import norm,iqr
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms


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



def remove_outliers(data,std_no):
    '''
        Based on "Data Outlier Detection using the Chebyshev Theorem",
        Brett G. Amidan, Thomas A. Ferryman, and Scott K. Cooley

        Based on the lecture slide of SEE at HBRS

        Reference:
        https://kyrcha.info/2019/11/26/data-outlier-detection-using-the-chebyshev-theorem-paper-review-and-online-adaptation


        Parameters
        -----------
        data -- A numpy array of discrete or continuous data
        std_no    -- No of deviatations to remove

        Returns
        -----------
        filtered_data: Data with outliers removed
    '''

    ## Create an list to store new filered data
    filtered_data_forward=np.array([])
    filtered_data_left=np.array([])
    filtered_data_right=np.array([])

    
    ## Loop between all three motion types
    for motion_index in [[0,3],[3,6],[6,9]]:
        
        current_motion=data[:,motion_index[0]:motion_index[1]]
        
    


        ##Loop between x and y
        
        for index in [0,1]:
            current_direction=current_motion[:,index]
            
            ##Mean and standard deviation of data
            mu = np.mean(current_direction)
            sigma = np.std(current_direction)

            ## k is the no of standard deviations
          
            k=std_no
            ## Outlier detection value upper bound
            odv1u = mu + k * sigma
            ## Outlier  detection value lower bound
            odv1l = mu - k * sigma
            ##Filter out outliers from upper and lower bounds
            final_data = current_direction[np.where(current_direction <= odv1u)[0]]
            final_data = final_data[np.where(final_data >= odv1l)[0]]



            

            ## If data is forward motion

            if motion_index[0]==0:

                

                ## Initialise the x values 
                if index==0:
                   

                    filtered_data_forward=final_data                                



                ## Adding the y values 
                else:                    

                    ## Check if x values are less than y values if so filter out the exrta y values to 
                    ## make the vectors equal in length
                    if len(filtered_data_forward)<= len(final_data):

                        print(f'flter {filtered_data_forward.shape} and final{final_data.shape}')

                        filtered_data_forward=np.vstack((filtered_data_forward,final_data[0:len(final_data)]))
                        filtered_data_forward=np.transpose(filtered_data_forward)                        
                            

                    ## Check if y values are less than x values if so filter out the extra x values to 
                    ## make the vectors equal in length
                    else:
                        filtered_data_forward=np.vstack((filtered_data_forward[0:len(final_data)],final_data))
                        filtered_data_forward=np.transpose(filtered_data_forward) 
                        

            ## If data is left motion

            if motion_index[0]==3:

                ## Initialise the x values 
                if index==0:
                    
                    filtered_data_left=final_data

                ## Adding the y values 
                else:                    

                    ## Check if x values are less than y values if so filter out the exrta y values to 
                    ## make the vectors equal in length
                    if len(filtered_data_left)<= len(final_data):
                        filtered_data_left=np.vstack((filtered_data_left,final_data[0:len(final_data)]))
                        filtered_data_left=np.transpose(filtered_data_left)
                        

                    ## Check if y values are less than x values if so filter out the extra x values to 
                    ## make the vectors equal in length
                    else:
                        filtered_data_left=np.vstack((filtered_data_left[0:len(final_data)],final_data))
                        filtered_data_left=np.transpose(filtered_data_left)
                        
            

            ## If data is forward right

            if motion_index[0]==6:

                ## Initialise the x values 
                if index==0:
                    
                    filtered_data_right=final_data

                ## Adding the y values 
                else:                    

                    ## Check if x values are less than y values if so filter out the exrta y values to 
                    ## make the vectors equal in length
                    if len(filtered_data_right)<= len(final_data):
                        filtered_data_right=np.vstack((filtered_data_right,final_data[0:len(final_data)]))
                        filtered_data_right=np.transpose(filtered_data_right)
                    
                        

                    ## Check if y values are less than x values if so filter out the extra x values to 
                    ## make the vectors equal in length
                    else:
                        filtered_data_right=np.vstack((filtered_data_right[0:len(final_data)],final_data))
                        filtered_data_right=np.transpose(filtered_data_right)  
                        




    ## Looking for vector with shortest lenght        
    min_motion_lenth=np.min([len(filtered_data_forward),len(filtered_data_left),len(filtered_data_right)])

    ## Resize all accoding to min length
    filtered_data=np.hstack((filtered_data_forward[0:min_motion_lenth,:],filtered_data_left[0:min_motion_lenth,:],filtered_data_right[0:min_motion_lenth,:]))
 
    

    return filtered_data


def general_outlier_removal(data, pp1, pp2):
    '''

        This is the general 2 stage outlier removal
        This is not used for this assignment
        Based on "Data Outlier Detection using the Chebyshev Theorem",
        Brett G. Amidan, Thomas A. Ferryman, and Scott K. Cooley

        Based on the lecture slide of SEE at HBRS

        Reference:
        https://kyrcha.info/2019/11/26/data-outlier-detection-using-the-chebyshev-theorem-paper-review-and-online-adaptation


        Parameters
        -----------
        data -- A numpy array of discrete or continuous data
        pp1 -- likelihood of expected outliers (e.g. 0.1, 0.05 , 0.01)
        pp2 -- final likelihood of real outliers (e.g. 0.01, 0.001 , 0.0001)

        Returns
        -----------
        filtered_data: Data with outliers removed
    '''

    ## Create an list to store new filered data
    filtered_data_forward=np.array([])
    filtered_data_left=np.array([])
    filtered_data_right=np.array([])

    
    ## Loop between all three motion types
    for motion_index in [[0,3],[3,6],[6,9]]:
        
        current_motion=data[:,motion_index[0]:motion_index[1]]
        
    


        ##Loop between x and y
        
        for index in [0,1]:
            current_direction=current_motion[:,index]
            
            ##Mean and standard deviation of data
            mu1 = np.mean(current_direction)
            sigma1 = np.std(current_direction)

            ## k is the no of standard deviations
            k = 1./ np.sqrt(pp1)
            ## Outlier detection value upper bound
            odv1u = mu1 + k * sigma1
            ## Outlier  detection value lower bound
            odv1l = mu1 - k * sigma1
            ##Filter out outliers from upper and lower bounds
            new_data = current_direction[np.where(current_direction <= odv1u)[0]]
            new_data = new_data[np.where(new_data >= odv1l)[0]]

            ## Second stage of filternig with a tighter bound 
            mu2 = np.mean(new_data)
            sigma2 = np.std(new_data)
            k = 1./ np.sqrt(pp2)
            odv2u = mu2 + k * sigma2
            odv2l = mu2 - k * sigma2

            ## Final filtered data after tighter bounds
            final_data = new_data[np.where(new_data <= odv2u)[0]]
            final_data = new_data[np.where(final_data >= odv2l)[0]]
            
            ## If data is forward motion

            if motion_index[0]==0:

                

                ## Initialise the x values 
                if index==0:
                   

                    filtered_data_forward=final_data                                



                ## Adding the y values 
                else:                    

                    ## Check if x values are less than y values if so filter out the exrta y values to 
                    ## make the vectors equal in length
                    if len(filtered_data_forward)<= len(final_data):

                        filtered_data_forward=np.vstack((filtered_data_forward,final_data[0:len(final_data)]))
                        filtered_data_forward=np.transpose(filtered_data_forward)                        
                            

                    ## Check if y values are less than x values if so filter out the extra x values to 
                    ## make the vectors equal in length
                    else:
                        filtered_data_forward=np.vstack((filtered_data_forward[0:len(final_data)],final_data))
                        filtered_data_forward=np.transpose(filtered_data_forward) 
                        

            ## If data is left motion

            if motion_index[0]==3:

                ## Initialise the x values 
                if index==0:
                    
                    filtered_data_left=final_data

                ## Adding the y values 
                else:                    

                    ## Check if x values are less than y values if so filter out the exrta y values to 
                    ## make the vectors equal in length
                    if len(filtered_data_left)<= len(final_data):
                        filtered_data_left=np.vstack((filtered_data_left,final_data[0:len(final_data)]))
                        filtered_data_left=np.transpose(filtered_data_left)
                        

                    ## Check if y values are less than x values if so filter out the extra x values to 
                    ## make the vectors equal in length
                    else:
                        filtered_data_left=np.vstack((filtered_data_left[0:len(final_data)],final_data))
                        filtered_data_left=np.transpose(filtered_data_left)
                        
            

            ## If data is forward right

            if motion_index[0]==6:

                ## Initialise the x values 
                if index==0:
                    
                    filtered_data_right=final_data

                ## Adding the y values 
                else:                    

                    ## Check if x values are less than y values if so filter out the exrta y values to 
                    ## make the vectors equal in length
                    if len(filtered_data_right)<= len(final_data):
                        filtered_data_right=np.vstack((filtered_data_right,final_data[0:len(final_data)]))
                        filtered_data_right=np.transpose(filtered_data_right)
                    
                        

                    ## Check if y values are less than x values if so filter out the extra x values to 
                    ## make the vectors equal in length
                    else:
                        filtered_data_right=np.vstack((filtered_data_right[0:len(final_data)],final_data))
                        filtered_data_right=np.transpose(filtered_data_right)  
                        




    ## Looking for vector with shortest lenght        
    min_motion_lenth=np.min([len(filtered_data_forward),len(filtered_data_left),len(filtered_data_right)])

    ## Resize all accoding to min length
    filtered_data=np.hstack((filtered_data_forward[0:min_motion_lenth,:],filtered_data_left[0:min_motion_lenth,:],filtered_data_right[0:min_motion_lenth,:]))
 
    

    return filtered_data
    
def gaussian_ellipsis(data): 
    '''
    Print the confidence ellipsis of the data we collected

    Reference
    https://matplotlib.org/devdocs/gallery/statistics/confidence_ellipse.html
    http://www.cs.utah.edu/~tch/CS6640F2020/resources/How%20to%20draw%20a%20covariance%20error%20ellipse.pdf
    Parameters
    -----------
    data : The data measurements ->np.array
    '''

    ##Looping through the indexes of all three motions
    for motion_index in [[0,3],[3,6],[6,9]]:
        
        current_motion=data[:,motion_index[0]:motion_index[1]]
        x=current_motion[:,0]
        y=current_motion[:,1]

        cov=np.cov(x,y)

        





def gaussian_distribution(data):
    '''
    Take in a data set and fit to gaussian


    Parameters
    ----------
    data : has the form [[forward],[left],[right]]
    

    
    '''

    ## Looping through the indexes of forward,left and right motions
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
            plot_title_x="Left data for x"
            plot_title_y="Left data for y"

        if motion_index[0]==6:
            plot_title_x="Right data for x"
            plot_title_y="Right data for y"

   

        gaus_figures=plt.figure(figsize=(12,8))
        ax1=gaus_figures.add_subplot(121)
        ax1.plot(x_range,norm.pdf(x_range,x_mean,x_std),label='Gaussian')
        ax1.hist(current_data[:,0],bins=x_n_bins,ec='black',density=True,label='Histogram')
        ax1.set(title=plot_title_x,xlabel='Data',ylabel='P(x)')
        ax1.grid()
        ax1.legend()

        ax2=gaus_figures.add_subplot(122)
        ax2.plot(y_range,norm.pdf(y_range,y_mean,y_std),label='Gaussian')
        ax2.hist(current_data[:,1],bins=y_n_bins,ec='black',density=True,label='Histogram')
        ax2.set(title=plot_title_y,xlabel='Data',ylabel='P(x)')
        ax2.grid()
        ax2.legend() 
       
        plt.show()

   

    




if __name__=='__main__':
    group_4_data,other_group_data=load_data()
    all_group_data=np.vstack((group_4_data,other_group_data))
    
    fitered_all_data=remove_outliers(all_group_data,2)
 
    gaussian_distribution(fitered_all_data)