#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np


# In[7]:


def get_unit_vector(x1, y1, x2, y2):
    
    unit_vector = np.array([x1 - x2, y1 - y2]) / np.linalg.norm(np.array([x1 - x2, y1 - y2]))
    
    return unit_vector

def get_angle(unit_vec1, unit_vec2):
    
    angle = np.rad2deg(np.arctan(np.dot(unit_vec1, unit_vec2)))
    
    return angle

get_angle(get_unit_vector(0,0,3,3), get_unit_vector(3,3,4,4))


# In[ ]:




