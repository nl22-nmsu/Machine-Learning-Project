Ensure "WindSpeed_data.xls" is in same folder as main.py and other supporting files

This dataset was generated using a matlab code that modeled a UAS flying in a controlled environment
with the specified wind speed conditions. 

To run code:
python main.py

This code will run all three algorithms (knn, svm, dtr) and will output the results of all
three algorithms for easy comparison. 

Orginization of the code base:
The main.py code is the main code file that imports all the supporting functions which are used to load the data, 
split the data into train and testing subsets, and call the regression algorithms (K-nearest neighbors, 
decision tree, and support vector machine). 
