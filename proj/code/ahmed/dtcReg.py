# -*- coding: utf-8 -*-
"""
Created on Wed Apr  7 16:08:42 2021

@author: Natha
"""
from sklearn.tree import DecisionTreeRegressor as DTC

def dtcReg(X_train,X_test,y_train,y_test):
    dtcRegressor = DTC()
    dtcRegressor.fit(X_train, y_train)
    acc = dtcRegressor.score(X_train, y_train)
    return acc