# -*- coding: utf-8 -*-
"""
Created on Mon Mar 29 14:56:52 2021

@author: Natha
"""
def createFeatures(X):
    #total_pos = math.sqrt(x_position[j] ** 2 + y_position[j] ** 2 + z_position[j] ** 2)
    #total_att = math.sqrt(droll[j] ** 2 + dpitch[j] ** 2 + dyaw[j] ** 2)
    #total_cont = math.sqrt(u_one[j] ** 2 + u_two[j] ** 2 + u_three[j] ** 2+u_four[j]**2)
    
    
    
    
    
    #uone_mean= statistics.mean(u_one)
    #utwo_mean = statistics.mean(u_two)
    #uthree_mean =statistics.mean(u_three)
    #ufour_mean = statistics.mean(u_four)
    
    
    
    # ======================= IMPORTANT ================================
    # This is not how the paper averages the values, needs to be revised
    #===================================================================
    
    # #Calculating all the means of the dataset
    # avg_x = statistics.mean(x_position)
    # avg_y = statistics.mean(y_position)
    # avg_z = statistics.mean(z_position)
    # avg_roll = statistics.mean(abs(droll))
    # avg_pitch = statistics.mean(abs(dpitch))
    # avg_yaw = statistics.mean(abs(dyaw))
    
    # #Calculating variances for the attributes
    # vari_x = statistics.variance(x_position)
    # vari_y = statistics.variance(y_position)
    # vari_z = statistics.variance(z_position)
    # vari_roll = statistics.mean(abs(droll))
    # vari_pitch = statistics.mean(abs(dpitch))
    # vari_yaw = statistics.mean(abs(dyaw))
    
    #print("x average:", avg_x)
    
    #=====================================================================
    return X