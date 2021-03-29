# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 12:31:14 2021

@author: Nathan
"""
from sklearn.neighbors import KNeighborsRegressor
import numpy as np

def knnRegressor(X_train, X_test, y_train, y_test, num):
    error_rate = []
    knnr = KNeighborsRegressor(n_neighbors= num)
    knnr.fit(X_train, y_train)
    r_squared = knnr.score(X_test,y_test)
    y_pred = knnr.predict(X_test)
    error_rate.append(np.mean(y_pred != y_test))
    
    return r_squared, error_rate
