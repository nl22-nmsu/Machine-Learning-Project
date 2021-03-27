# -*- coding: utf-8 -*-
"""
Created on Fri Mar  5 21:11:15 2021

@author: Natha
"""
from sklearn.model_selection import train_test_split

# Time stamped data
def testSplit(X_sample, y_sample,testSize):
    X_train, X_test, y_train, y_test = train_test_split(X_sample, y_sample,test_size = testSize, 
                                                        random_state = 1, 
                                                        stratify = y_sample)
    return X_train, X_test, y_train, y_test