"""
Created on Sat Apr  3 21:08:36 2021

@author: Nathan
"""
from sklearn.linear_model import LinearRegression as LR
from sklearn.metrics import max_error
from sklearn.metrics import accuracy_score
import numpy as np

def linearReg(X_train, X_test, y_train, y_test):
    lr = LR().fit(X_train, y_train)
    y_pred = lr.predict(X_test)
    acc = lr.score(X_train,y_train)
    return acc