#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
from scipy.stats import norm,iqr,chisquare
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


    ## Team 1 has same format as us so no need to convert
    
    forward_measurements1=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_1_reading/front_measurement.csv',delimiter=',',skip_header=1)
    left_measurements1=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_1_reading/left_measurement.csv',delimiter=',',skip_header=1)
    right_measurements1=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_1_reading/right_measurement.csv',delimiter=',',skip_header=1)

    ## Team 2 have bad data . Will not use it

    forward_measurements2=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_2_reading/Forward.csv',delimiter=',',skip_header=1)
    left_measurements2=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_2_reading/Left.csv',delimiter=',',skip_header=1)
    right_measurements2=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_2_reading/Right.csv',delimiter=',',skip_header=1)


    ## Team 3 has has taken clock wise positive so have to convert it to anti-clock wise positive to fit our teams definition of coordinate 
    forward_measurements3=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_3_reading/straight_end_poses.csv',delimiter=',',skip_header=1)
    forward_measurements3=forward_measurements3[:,1:]
    left_measurements3=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_3_reading/left_end_poses.csv',delimiter=',',skip_header=1)
    left_measurements3=left_measurements3[:,1:]
    right_measurements3=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_3_reading/right_end_poses.csv',delimiter=',',skip_header=1)
    right_measurements3=right_measurements3[:,1:]


    
    ## We are team 4


    ##Team 5 has a different format in csv therefore have to filter it and had to convert angles to starting from y axis and anticlockwise positive

    all_measurements5=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_5_reading/Manual_measurement_group5.csv',delimiter=',',skip_header=1)
    forward_measurements5=all_measurements5[1:22,5:8]
    left_measurements5=all_measurements5[1:22,1:4]
    right_measurements5=all_measurements5[1:22,9:14]



    ## Making group 3 and group 5 orienation anti-clock wise poisitive
    for index in range(len(forward_measurements3)):
        forward_measurements3[index,2]=-1*forward_measurements3[index,2]
        left_measurements3[index,2]=-1*left_measurements3[index,2]
        right_measurements3[index,2]=-1*right_measurements3[index,2]

        forward_measurements5[index,2]=-1*forward_measurements5[index,2]
        left_measurements5[index,2]=-1*left_measurements5[index,2]
        right_measurements5[index,2]=-1*right_measurements5[index,2]



    
    ## Team 6 has distances in angles in rads therefore have to convert it to deg
    
    forward_measurements6=np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_6_reading/straight_readings.csv',delimiter=' ',skip_header=1)
    left_measurements6=   np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_6_reading/left_readings.csv',    delimiter='',skip_header=1)
    right_measurements6=  np.genfromtxt('../data/Assignment_1_2/Other_team_data/group_6_reading/right_readings.csv',   delimiter='',skip_header=1)
    
    for index in range(len(forward_measurements6)):
        forward_measurements6[index,2]=np.rad2deg(forward_measurements6[index,2])-90.0
        left_measurements6[index,2]=np.rad2deg(left_measurements6[index,2])-90.0
        right_measurements6[index,2]=np.rad2deg(right_measurements6[index,2])-90.0


    ##Combine all group data

    
    # forward_measurements_all=np.concatenate((forward_measurements1,forward_measurements3,forward_measurements5,forward_measurements6),axis=0)

    
    # left_measurements_all=np.concatenate((left_measurements1,left_measurements3,left_measurements5,left_measurements6),axis=0)
    
    
    # right_measurements_all=np.concatenate((right_measurements1,right_measurements3,right_measurements5,right_measurements6),axis=0)

    forward_measurements_all=np.concatenate((forward_measurements1,forward_measurements3,forward_measurements6),axis=0)

    
    left_measurements_all=np.concatenate((left_measurements1,left_measurements3,left_measurements6),axis=0)
    
    
    right_measurements_all=np.concatenate((right_measurements1,right_measurements3,right_measurements6),axis=0)



   

    
    group_4_data=np.hstack((forward_measurements4,left_measurements4,right_measurements4))
    group_all_data=np.hstack((forward_measurements_all,left_measurements_all,right_measurements_all))
    

        

    
    
    
    return group_4_data,group_all_data



def remove_outliers(data,std_no,plot_title):
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
    ori_data_count=len(data)

     ##Mean and standard deviation of data
    mu = np.mean(data)
    sigma = np.std(data)

    ## k is the no of standard deviations
    
    k=std_no
    ## Outlier detection value upper bound
    odv1u = mu + k * sigma
    ## Outlier  detection value lower bound
    odv1l = mu - k * sigma
    ##Filter out outliers from upper and lower bounds

    stage1_data = data[np.where(data <= odv1u)[0]]
    stage1_data = stage1_data[np.where(stage1_data >= odv1l)[0]]

     ##Mean and standard deviation of data
    mu2 = np.mean(stage1_data)
    sigma2 = np.std(stage1_data)

    ## k is the no of standard deviations
    
    k=std_no
    ## Outlier detection value upper bound
    odv2u = mu2 + k * sigma2
    ## Outlier  detection value lower bound
    odv2l = mu2 - k * sigma2
    ##Filter out outliers from upper and lower bounds



    final_data = data[np.where(stage1_data <= odv2u)[0]]
    final_data = final_data[np.where(final_data >= odv2l)[0]]

    ##No of data poitns after filter
    fil_data_count=len(final_data)


    print(f'Filtered data summary for {plot_title}\nStart data points: {ori_data_count} Outliers:{ori_data_count-fil_data_count}\n')


    return final_data
    


def confidence_ellipse(x, y, ax, n_std=3.0, facecolor='none', **kwargs):
    """
    Create a plot of the covariance confidence ellipse of *x* and *y*.

    Parameters
    ----------
    x, y : array-like, shape (n, )
        Input data.

    ax : matplotlib.axes.Axes
        The axes object to draw the ellipse into.

    n_std : float
        The number of standard deviations to determine the ellipse's radiuses.

    **kwargs
        Forwarded to `~matplotlib.patches.Ellipse`

    Returns
    -------
    matplotlib.patches.Ellipse
    """
    if x.size != y.size:
        raise ValueError("x and y must be the same size")

    cov = np.cov(x, y)
    pearson = cov[0, 1]/np.sqrt(cov[0, 0] * cov[1, 1])
    # Using a special case to obtain the eigenvalues of this
    # two-dimensionl dataset.
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2,
                      facecolor=facecolor, **kwargs)

    # Calculating the stdandard deviation of x from
    # the squareroot of the variance and multiplying
    # with the given number of standard deviations.
    scale_x = np.sqrt(cov[0, 0]) * n_std
    mean_x = np.mean(x)

    # calculating the stdandard deviation of y ...
    scale_y = np.sqrt(cov[1, 1]) * n_std
    mean_y = np.mean(y)

    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(mean_x, mean_y)

    ellipse.set_transform(transf + ax.transData)
    return ax.add_patch(ellipse)


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

        

def PCA(X: np.array, k: int) -> np.array:
    '''
    X : Data
    k : No of dimensio
    
    
    '''
    cov = np.cov(X)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    k_largest_eigenvalue_idx = np.argsort(eigenvalues)[::-1][0:k]
    W = eigenvectors[:,k_largest_eigenvalue_idx]
    Y = W.T.dot(X)

    return Y





def gaussian_distribution(data,plot_title):
    '''
    Take in a data set and compare its histogram 
    with a gausian usnig matplotlib


    Parameters
    ----------
    data : data of a particual motion
    plot_title  : Title of the plot
    

    
    '''

         

    data_mean=np.mean(data)
    data_std =np.std(data)
  
        ## Freedman–Diaconis rule for calculating number of bins in a histogram
        ## https://en.wikipedia.org/wiki/Freedman%E2%80%93Diaconis_rule
    n_bins=int(round((np.max(data)-np.min(data))/(2*iqr(data)*len(data)**(-1/3))))
    data_range=np.arange(np.min(data),np.max(data),0.1)

    gaus_figures=plt.figure(figsize=(6,4))
    ax1=gaus_figures.add_subplot(111)
    ax1.plot(data_range,norm.pdf(data_range,data_mean,data_std),label='Gaussian')
    ax1.hist(data,bins=n_bins,ec='black',density=True,label='Histogram')
    ax1.set(title=plot_title,xlabel='Data',ylabel='P(x)')
    ax1.grid()
    ax1.legend()

    plt.show()


def chi_square(data):
    '''
    
    
    '''

    observed_data=data
    n_bins=int(round((np.max(data)-np.min(data))/(2*iqr(data)*len(data)**(-1/3))))
    observed_freq,_=np.histogram(observed_data,bins=n_bins)
    mu = np.mean(observed_data)
    std= np.std(observed_data)
    expected_data=np.random.normal(mu,std,len(data))
    expected_freq,_ = np.histogram(expected_data, bins=len(observed_freq))

    

    return chisquare(observed_freq,expected_freq)





#########################
##Action functions
########################

def compare_gauss_vs_hist(data):
    '''
    Action funcion to run plotting the gaussian histograms together
    Loop throug all motions and direcions while removing outliers and comparing its gaussian
    
    '''


    for motion_index in range(0,9):
        current_motion=data[:,motion_index]
        
        if motion_index==0:
            plot_title="Forward motion x direction"            

        if motion_index==1:
            plot_title="Forward motion y direction"

        if motion_index==2:
            plot_title="Forward motion theta"

        if motion_index==3:
            plot_title="Left motion x direction"        

        if motion_index==4:
            plot_title="Left motion y direction"

        if motion_index==5:
            plot_title="Left motion theta direction"

        if motion_index==6:
            plot_title="Right motion x direction"

        if motion_index==7:
            plot_title="Right motion y direction"

        if motion_index==8:
            plot_title="Right motion theta direction"



        fitered_data=remove_outliers(current_motion,2,plot_title) 
        # print(f'Length of current{len(current_motion)} and filter {len(fitered_data)}')
        ## The cause for shitty outliers could be that group 5 with weird data
        
        gaussian_distribution(fitered_data,plot_title)
        


def plot_ellipsis(data):
    '''
    Action function to run ellipsis plots
    
    '''
        



    for motion_index in range(0,9,3):
            current_motion=all_group_data[:,motion_index]
            
            if motion_index==0:
                plot_title="Forward motion"            

           
            if motion_index==3:
                plot_title="Left motion"        


            if motion_index==6:
                plot_title="Right motion"

            

            fig, ax_nstd = plt.subplots(figsize=(6, 6))
            ax_nstd.axvline(c='grey', lw=1)
            ax_nstd.axhline(c='grey', lw=1)

            x= data[:,motion_index]
            x_range=[np.min(x),np.max(x)]
            y= data[:,motion_index+1]
            y_range=[np.min(y),np.max(y)]
            ax_nstd.scatter(x, y, s=0.5)

            mu=[np.mean(x),np.mean(y)]

            confidence_ellipse(x, y, ax_nstd, n_std=1,
                            label=r'$1\sigma$', edgecolor='firebrick')
            confidence_ellipse(x, y, ax_nstd, n_std=2,
                            label=r'$2\sigma$', edgecolor='fuchsia', linestyle='--')
            confidence_ellipse(x, y, ax_nstd, n_std=3,
                            label=r'$3\sigma$', edgecolor='blue', linestyle=':')

            ax_nstd.scatter(mu[0], mu[1], c='red', s=3)    
            ax_nstd.set(title=plot_title,xlim=(x_range[0]-4,x_range[1]+2),ylim=(y_range[0]-2,y_range[1]+2),xlabel='x(cm)',ylabel='y(cm)')
            
            ax_nstd.legend()
            ax_nstd.grid()
            plt.show()

   

def compute_PCA(data):
    '''
    Reduce the dimensionality of data 
    and plot the ellipse of it 
    
    '''


    for motion_index in range(0,9,3):
        
        current_motion=data[:,motion_index:motion_index+3]
        
        
        
        if motion_index==0:
            plot_title="Forward motion distribution after PCA"            

        
        if motion_index==3:
            plot_title="Left motion distribution after PCA"        


        if motion_index==6:
            plot_title="Right motion distribution after PCA"



        current_motion=np.transpose(current_motion)        
        Y = PCA(current_motion,2)   
        Y=np.transpose(Y)

        fig, ax_nstd = plt.subplots(figsize=(6, 6))
        ax_nstd.axvline(c='grey', lw=1)
        ax_nstd.axhline(c='grey', lw=1)

        x=Y[:,0]
        x_range=[np.min(x),np.max(x)]
        y=Y[:,1]
        y_range=[np.min(y),np.max(y)]
        ax_nstd.scatter(x, y, s=0.5)

        mu=[np.mean(x),np.mean(y)]

        confidence_ellipse(x, y, ax_nstd, n_std=1,
                        label=r'$1\sigma$', edgecolor='firebrick')
        confidence_ellipse(x, y, ax_nstd, n_std=2,
                        label=r'$2\sigma$', edgecolor='fuchsia', linestyle='--')
        confidence_ellipse(x, y, ax_nstd, n_std=3,
                        label=r'$3\sigma$', edgecolor='blue', linestyle=':')

        ax_nstd.scatter(mu[0], mu[1], c='red', s=3)    
        ax_nstd.set(title=plot_title,xlim=(x_range[0]-10,x_range[1]+10),ylim=(y_range[0]-10,y_range[1]+20),xlabel='PC1',ylabel='PC2')
        
        ax_nstd.legend()
        ax_nstd.grid()
        plt.show()



def calculate_chi(data):
    '''
    
    
    '''


    for motion_index in range(0,9):
        current_motion=data[:,motion_index]
        
        if motion_index==0:
            plot_title="Forward motion x direction"            

        if motion_index==1:
            plot_title="Forward motion y direction"

        if motion_index==2:
            plot_title="Forward motion theta"

        if motion_index==3:
            plot_title="Left motion x direction"        

        if motion_index==4:
            plot_title="Left motion y direction"

        if motion_index==5:
            plot_title="Left motion theta direction"

        if motion_index==6:
            plot_title="Right motion x direction"

        if motion_index==7:
            plot_title="Right motion y direction"

        if motion_index==8:
            plot_title="Right motion theta direction"

        chi_stat,p_value=chi_square(current_motion)

        print(f'For motion {plot_title} chi stat is {chi_stat} and p value is {p_value}')

    
        
          


    

    

if __name__=='__main__':
    group_4_data,other_group_data=load_data()
    all_group_data=np.vstack((group_4_data,other_group_data))

    compare_gauss_vs_hist(all_group_data)
    plot_ellipsis(all_group_data)
    # compute_PCA(all_group_data)

    # calculate_chi(all_group_data)
   


    