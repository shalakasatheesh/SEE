#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import numpy as np
import matplotlib.pyplot as pyplot
import seaborn as sb
import os
import sys



def load_measurement_data():
    '''
    Import data from the measurement files
    
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
        with open(os.path.join('../data/forward_logs', filename), 'r') as f: # open in readonly mode
            current_filename='get_'+str(filename_counter)
            filename_counter+=1
            # print(current_filename)
            # print(f)
            if (filename_counter%2)==0:
                # np.append(forward_logs,np.genfromtxt(f,delimiter=',',skip_header=1),axis=0)
                # print(filename[-20:-4])
                a=10
            if filename_counter==3:
                print(filename)
                aa=np.genfromtxt(f,delimiter=',',skip_header=1)
                print(aa)
                # print(aa.shape)






forward,left,right=load_measurement_data()
load_log_data()