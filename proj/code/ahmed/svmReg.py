# -*- coding: utf-8 -*-
"""
Created on Wed Apr 28 19:30:51 2021

@author: Nathan
"""
from sklearn import svm
import time

def SVReg(X_train, X_test, y_train, y_test):
    svmReg = svm.SVR()
    ts = time.time()
    svmReg.fit(X_train, y_train)
    acc = svmReg.score(X_test, y_test)
    te = time.time()
    tTot = te-ts
    return acc, tTot