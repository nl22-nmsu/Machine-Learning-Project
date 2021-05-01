# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 10:31:54 2021

@author: Nathan
"""
# import numpy as np
# import pandas as pd
import LoadData
import splitData
import knnRegressor
import dtcReg
import svmReg
import addNoise
import createFeatures
# import argparse

import matplotlib.pyplot as plt

# parser = argparse.ArgumentParser(description='Takes in number of neighbors')

# parser.add_argument('-n','--number')
# arg = parser.parse_args()

print("Loading data...")
X,y = LoadData.LoadData()
X = createFeatures.createFeatures(X)
X_train, X_test, y_train, y_test = splitData.testSplit(X, y, 0.3)

#I suspect x,y,z should be bigger than roll, pitch, yaw in general: We need to characterize these
#Zeros means no noise is added
scaleFactor = [0, 0, 0, 0, 0, 0] #Change accordingly for x,y,z,roll,pitch,yaw
X_test_noisy = addNoise.addNoise(X_test, scaleFactor)
# print(X_test.iloc[0:4,:])
# print(X_test_noisy.iloc[0:4,:])

knnAccs = []
error_rates = []
timesKNN = []
maxN = 6 #N+1 
print('beginning fitting')
for i in range(1,maxN):
    numOfNeighbors = i#int(arg.number)
    knnAcc, error_rate, timeKNN = knnRegressor.knnRegressor(X_train, X_test_noisy, y_train, y_test, numOfNeighbors) #Calling funciton in knnRegressor
    knnAccs.append((knnAcc)*100)
    error_rates.append(error_rate)
    timesKNN.append(timeKNN)
print("KNN finished...\nBeginning DTR")

dtrAccs = []
timesDTR = []
maxD = 21 #N+1
for i in range(1, maxD):
    max_depth = i
    dtregAcc, timeDTR = dtcReg.dtcReg(X_train, X_test_noisy, y_train, y_test, maxDep=max_depth)
    dtrAccs.append(dtregAcc)
    timesDTR.append(timeDTR)
print("DTR finished...\nBeginning SVM")
svmAcc, timeSVM = svmReg.SVReg(X_train, X_test, y_train, y_test) #Using RBF kernel (default)
print("Accuracy of knn regression algorithm: %0.3f ===> Run time: %0.6f seconds"%((knnAcc*100), timeKNN))
print("Accuracy of decision tree regression algorithm: %0.3f ===> Run time: %0.6f seconds"%(dtregAcc*100, timeDTR))
print("Accuracy of support vector machine regression algorithm: %0.3f ===> Run time: %0.6f seconds"%(svmAcc*100, timeSVM))

#print("Error rate: ",error_rate)
fig, (ax1, ax2) = plt.subplots(1, 2)
fig.suptitle('Effects of Numbers of Neighbors Used on KNN')
ax1.plot(range(1,maxN), knnAccs)
ax1.set_title("Number of Neighbors vs. Accuracy")
ax1.set_xlabel("Number of Neighbors")
ax1.set_ylabel("Accuracy (%)")
ax2.plot(range(1,maxN), timesKNN)
ax2.set_title("Number of Neighbors vs. Time")
ax2.set_xlabel("Number of Neighbors")
ax2.set_ylabel("Time (seconds)")
plt.show()


fig2, (ax3, ax4) = plt.subplots(1, 2)
fig2.suptitle('Effects of Max Depth on DTR')
ax3.plot(range(1,maxD), dtrAccs)
ax3.set_title("Max Depth vs. Accuracy")
ax3.set_xlabel("Max Depth")
ax3.set_ylabel("Accuracy (%)")
ax4.plot(range(1,maxD), timesDTR)
ax4.set_title("Max Depth vs. Time")
ax4.set_xlabel("Max Depth")
ax4.set_ylabel("Time (seconds)")
plt.show()
