# -*- coding: utf-8 -*-
"""
Created on Wed Apr 28 19:30:51 2021

@author: Natha
"""
from sklearn import svm

def SVReg(X_train, X_test, y_train, y_test):
    svmReg = svm.SVR()
    svmReg.fit(X_train, Y_train)
    svm