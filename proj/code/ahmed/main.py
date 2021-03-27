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

X,y = LoadData.LoadData()

print("beginning data split")
X_train, X_test, y_train, y_test = splitData.testSplit(X, y, 0.3)
print("splitting complete")

print('beginning fitting')
numOfNeighbors = 1
knnAcc = knnRegressor.knnRegressor(X_train, X_test, y_train, y_test, numOfNeighbors)
print('beginning scorring')
print("Accuracy of knn regression algorithm:",knnAcc)