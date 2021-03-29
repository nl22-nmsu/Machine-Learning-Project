# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 10:31:54 2021

@author: Nathan
"""
import numpy as np
import pandas as pd
from sklearn.neighbors import KNeighborsRegressor
import LoadData
import splitData
import knnRegressor
import addNoise

X,y = LoadData.LoadData()

print("beginning data split")
X_train, X_test, y_train, y_test = splitData.testSplit(X, y, 0.3)
print("splitting complete")
#I suspect x,y,z should be bigger than roll, pitch, yaw in general: We need to characterize these
#Zeros means no noise is added
scaleFactor = [0, 0, 0, 0.4, 0.5, 0.6] #Change accordingly for x,y,z,roll,pitch,yaw
X_test_noisy = addNoise.addNoise(X_test, scaleFactor)
print(X_test.iloc[0:4,:])
print(X_test_noisy.iloc[0:4,:])

print('beginning fitting')
numOfNeighbors = 1
knnAcc = knnRegressor.knnRegressor(X_train, X_test, y_train, y_test, numOfNeighbors)
print('beginning scorring')
print("Accuracy of knn regression algorithm:",knnAcc)