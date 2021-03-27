import pandas as pd
import numpy as np
import math
import statistics
from sklearn.neighbors import KNeighborsRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
from sklearn.preprocessing import StandardScaler
import time
# Loading all data
start = time.time()
dfs = []
for i in range(8):
    dfs.append(pd.read_excel("WindSpeedData.xls", sheet_name=i-1))

#numpy.array((0),0)  - creating tuple, using numpy array
# Create a list for all attributes in the data.
mean_x = []
mean_y = []
mean_z = []
mean_roll = []
mean_pitch = []
mean_yaw = []
mean_u1 = []
mean_u2 = []
mean_u3 = []
mean_u4 = []

var_x = []
var_y =[]
var_z = []
var_roll = []
var_pitch = []
var_yaw = []

Pe = []
Ae = []
Cont_eff = []
mean_total_con = []
mean_total_pos = []
mean_total_att = []


for i in range(8):
# Collecting the attributes individually for analysis
    x_position = dfs[i]["X_Pos"]
    y_position = dfs[i]["Y_Pos"]
    z_position = dfs[i]["Z_Pos"]
    droll = dfs[i]["drone_roll"]
    dpitch = dfs[i]["pitch"]
    dyaw = dfs[i]["Yaw"]
    u_one =dfs[i]["u1"]
    u_two =dfs[i]["u2"]
    u_three =dfs[i]["yaw vel"]
    u_four =dfs[i]["u4"]

    iterator = 0
    for j in range(len(x_position)):
        total_pos = math.sqrt(x_position[j] ** 2 + y_position[j] ** 2 + z_position[j] ** 2)
        total_att = math.sqrt(droll[j] ** 2 + dpitch[j] ** 2 + dyaw[j] ** 2)
        total_cont = math.sqrt(u_one[j] ** 2 + u_two[j] ** 2 + u_three[j] ** 2+u_four[j]**2)


        Pe.append(total_pos)
        Ae.append(total_att)
        Cont_eff.append(total_cont)
        iterator = iterator + 1


    uone_mean= statistics.mean(u_one)
    utwo_mean = statistics.mean(u_two)
    uthree_mean =statistics.mean(u_three)
    ufour_mean = statistics.mean(u_four)

    tp_mean = statistics.mean(Pe)                     # mean of total position, control. Features 7,12,19
    ta_mean = statistics.mean(Ae)
    tcont_mean = statistics.mean(Cont_eff)


#Calculating all the means of the dataset
    avg_x = statistics.mean(x_position)
    avg_y = statistics.mean(y_position)
    avg_z = statistics.mean(z_position)
    avg_roll = statistics.mean(abs(droll))
    avg_pitch = statistics.mean(abs(dpitch))
    avg_yaw = statistics.mean(abs(dyaw))

#Calculating variances for the attributes
    vari_x = statistics.variance(x_position)
    vari_y = statistics.variance(y_position)
    vari_z = statistics.variance(z_position)
    vari_roll = statistics.mean(abs(droll))
    vari_pitch = statistics.mean(abs(dpitch))
    vari_yaw = statistics.mean(abs(dyaw))

#Append all the means  and variances of the different to 4 significant figures
    # Position features
    mean_x.append(round(avg_x,4))  #f1
    var_x.append(round(vari_x, 4))  #f2
    mean_y.append(round(avg_x,4))   #f3
    var_y.append(round(vari_y, 4))  #f4
    mean_z.append(round(avg_z,4))   #f5
    var_z.append(round(vari_z, 4))  #f6
    mean_total_pos.append(round(tp_mean, 4)) #f7

    #Euler Angles
    mean_roll.append(round(avg_roll,4))  #f8
    var_roll.append(round(vari_roll, 4)) #f9
    mean_pitch.append(round(avg_pitch,4)) #f10
    var_pitch.append(round(vari_pitch, 4)) #f11
    mean_yaw.append(round(avg_yaw, 4))  #f12
    var_yaw.append(round(vari_yaw, 4))  #f13
    mean_total_att.append(round(ta_mean, 4)) #f14

    #Control Efforts
    mean_u1.append(round(uone_mean,4)) #f15
    mean_u2.append(round(utwo_mean,4)) #f16
    mean_u3.append(round(uthree_mean,4)) #f17
    mean_u4.append(round(ufour_mean,4))  #f18
    mean_total_con.append(round(tcont_mean, 4)) #f19

#Create A new dataset of features 1-20
my_df = {'f1': pd.Series(mean_x), 'f2': pd.Series(var_x), 'f3':pd.Series(mean_y), 'f4': pd.Series(var_y),
      'f5':pd.Series(mean_z),'f6': pd.Series(var_z), 'f7': pd.Series(mean_total_pos),
      'f8':pd.Series(mean_roll), 'f9':pd.Series(var_roll), 'f10': pd.Series(mean_pitch),'f11': pd.Series(var_pitch),
      'f12': pd.Series(mean_yaw),'f13': pd.Series(var_yaw),'f14': pd.Series(mean_total_att),
      'f15': pd.Series(mean_u1), 'f16': pd.Series(mean_u2), 'f17': pd.Series(mean_u3), 'f18': pd.Series(mean_u4),
      'f19': pd.Series(mean_total_con),"f20": pd.Series([0,0.6,1.2,1.8,2.4,3,3.6,4.2])}


my_dataframe = pd.DataFrame(my_df)
print(my_dataframe)


X = my_dataframe.iloc[:,1:19]
y = my_dataframe.iloc[:,-1]  # Target Values

print(len(X))
print(len(y))
X_train, X_test, y_train,y_test = train_test_split(X,y,test_size =0.3,random_state=1)

sc =StandardScaler()
sc.fit(X_train)
Xtrain_std = sc.transform(X_train)
Xtest_std = sc.transform(X_test)

error_rate = []
for i in range(1,6):
    knn = KNeighborsRegressor(n_neighbors=i,p=1,metric='minkowski')
    knn.fit(Xtrain_std,y_train)
    y_pred = knn.predict(Xtest_std)
    error_rate.append(np.mean(y_pred != y_test))
    print(y_pred)

print(error_rate)
print("Minimum error:", min(error_rate), "at K =", error_rate.index(min(error_rate)))
#print('Accuracy: %0.3f' % accuracy_score(y_test, y_pred))
end = time.time()


'''
print(mean_x)
print(var_x)
print(mean_y)
print(var_y)
print(mean_z)
print(var_z)
print(mean_total_pos)
print(mean_roll)
print(var_roll)
print(mean_pitch)
print(var_pitch)
print(mean_yaw)
print(var_yaw)
print(mean_u1)
print(mean_u2)
print(mean_u3)
print(mean_u4)
print(mean_total_con)
'''
#Normalization is skipped - Data needs multiple data of the same experiment for the same experiment

#Training program with KNN Scikit learn

end = time.time()
print("Run time", end-start)