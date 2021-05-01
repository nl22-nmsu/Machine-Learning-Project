# -*- coding: utf-8 -*-
"""
Created on Wed Apr  7 16:08:42 2021

@author: Nathan
"""
from sklearn.tree import DecisionTreeRegressor as DTC
import time

def dtcReg(X_train,X_test,y_train,y_test, maxDep):
    dtcRegressor = DTC(max_depth= maxDep)
    ts = time.time()
    dtcRegressor.fit(X_train, y_train)
    acc = dtcRegressor.score(X_train, y_train)
    te = time.time()
    tTot = te-ts
    return acc, tTot