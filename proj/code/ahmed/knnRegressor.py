# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 12:31:14 2021

@author: Nathan
"""
from sklearn.neighbors import KNeighborsRegressor

def knnRegressor(X_train, X_test, y_train, y_test, num):
    neigh = KNeighborsRegressor(n_neighbors= num)
    neigh.fit(X_train, y_train)
    r_squared = neigh.score(X_test,y_test)
    return r_squared
