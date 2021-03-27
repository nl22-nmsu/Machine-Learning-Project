import pandas as pd
import numpy as np
import math
import statistics
import LoadData
from sklearn.neighbors import KNeighborsRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
from sklearn.preprocessing import StandardScaler
import time
# Loading all data
start = time.time()
#dfs = []
#for i in range(8):
 #   dfs = pd.DataFrame(pd.read_excel("WindSpeed_Data.xls", sheet_name=i-1))

#numpy.array((0),0)  - creating tuple, using numpy array
# Create a list for all attributes in the data.

print("loading data...")
X, y = LoadData.LoadData()
print(X)
print(y)
print("loaded data")

# Collecting the attributes individually for analysis
x_position = X.iloc[:, 0]
y_position = X.iloc[:, 1]
z_position = X.iloc[:, 2]
droll = X.iloc[:, 3]
dpitch = X.iloc[:, 4]
dyaw = X.iloc[:, 5]

#u_one =dfs[i]["u1"]
#u_two =dfs[i]["u2"]
#u_three =dfs[i]["yaw vel"]
#u_four =dfs[i]["u4"]




#total_pos = math.sqrt(x_position[j] ** 2 + y_position[j] ** 2 + z_position[j] ** 2)
#total_att = math.sqrt(droll[j] ** 2 + dpitch[j] ** 2 + dyaw[j] ** 2)
#total_cont = math.sqrt(u_one[j] ** 2 + u_two[j] ** 2 + u_three[j] ** 2+u_four[j]**2)





#uone_mean= statistics.mean(u_one)
#utwo_mean = statistics.mean(u_two)
#uthree_mean =statistics.mean(u_three)
#ufour_mean = statistics.mean(u_four)




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











X_train, X_test, y_train,y_test = train_test_split(X,y,test_size =0.3,random_state=1)

sc =StandardScaler()
sc.fit(X_train)
Xtrain_std = sc.transform(X_train)
Xtest_std = sc.transform(X_test)

error_rate = []

knn = KNeighborsRegressor(n_neighbors=2, p=1, metric='minkowski')
knn.fit(Xtrain_std, y_train)
y_pred = knn.predict(Xtest_std)
error_rate.append(np.mean(y_pred != y_test))
print(y_pred)

#print(error_rate)
#print("Minimum error:", min(error_rate), "at K =", error_rate.index(min(error_rate)))
print('Accuracy:', knn.score(X_test, y_test))

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