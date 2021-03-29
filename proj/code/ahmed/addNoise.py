# -*- coding: utf-8 -*-
"""
Created on Mon Mar 29 13:55:58 2021

@author: Natha
"""
import numpy as np
import pandas as pd
def addNoise(X, scaleFactor):
    #X_pos = X.iloc[:,0:3] #x,y,z positions
    #Create columns of length l using random then add that column to the column for X
    noise = np.zeros(X.shape)
    for i in range(len(X.iloc[0,:])):
        if len(scaleFactor) == len(X.iloc[0,:]):
            noise[:, i] = np.random.normal(loc=0.0, scale = scaleFactor[i], size = len(X.iloc[:,0]))
        else:
            print("Error: Scale vector should be same length as number of columns in X")
            break
    #print(noise[0:4,:])
    X_new = X + noise
    return X_new
