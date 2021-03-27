# -*- coding: utf-8 -*-
"""
Created on Fri Mar 12 19:21:29 2021

@author: Nathan
"""
import pandas as pd
import numpy as np


def LoadData():
    #TODO: Consider putting this stuff in a loop for easier and more robust importing
    df1 = pd.read_excel("WindSpeed_Data.xls", sheet_name='0mps')    #0 m/s        #Import the worksheets individually
    df2 = pd.read_excel("WindSpeed_Data.xls", sheet_name='0.6mps')
    df3 = pd.read_excel("WindSpeed_Data.xls", sheet_name='1.2mps')   #1.2m/s
    df4 = pd.read_excel("WindSpeed_Data.xls", sheet_name='1.8mps')   #1.8m/s
    df5 = pd.read_excel("WindSpeed_Data.xls", sheet_name='2.4mps')   #2.4 m/s
    df6 = pd.read_excel("WindSpeed_Data.xls", sheet_name='3mps')   #3.0 m/s
    df7 = pd.read_excel("WindSpeed_Data.xls", sheet_name='3.6mps')   #3.6 m/s
    df8 = pd.read_excel("WindSpeed_Data.xls", sheet_name='4.2mps')   #4.2 m/s
    
    y1 = np.zeros((len(df1), 1))
    y_ones2 = np.ones((len(df2), 1))
    y_ones3 = np.ones((len(df3), 1))
    y_ones4 = np.ones((len(df4), 1))
    y_ones5 = np.ones((len(df5), 1))
    y_ones6 = np.ones((len(df6), 1))
    y_ones7 = np.ones((len(df7), 1))
    y_ones8 = np.ones((len(df8), 1))
    y2 = y_ones2*0.6
    y3 = y_ones3*1.2
    y4 = y_ones4*1.8
    y5 = y_ones5*2.4
    y6 = y_ones6*3.0
    y7 = y_ones7*3.6
    y8 = y_ones8*4.2
    
    df1['speed']=y1
    df2['speed']=y2
    df3['speed']=y3
    df4['speed']=y4
    df5['speed']=y5
    df6['speed']=y6
    df7['speed']=y7
    df8['speed']=y8
    
    fullData = pd.concat([df1, df2, df3, df4, df5, df6, df7, df8])
    X = fullData.iloc[:, 1:7]
    y = fullData.iloc[:, -1]
    X = X.replace('NaN', np.nan)
    X = X.dropna(axis=0, how="any")
    return X, y