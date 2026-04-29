import numpy as np
import math
import h5py
import os
import pandas as pd
import sys



# First we have to load the dataset:
# we use pd.read_excel documentation to understand sheet_name = tab, and engine

#Read excel file
file = pd.read_excel('Scats Data October 2006.xls', sheet_name=1, engine='calamine')


#Select columns to be used as input
input_idxs = [1, 9]
columns_names = ['LOCATION', 'DATE']

#Get column names- from column header
column_names = file.columns[input_idxs]

#List of column values- iterates over the selected column names and appends the values into a NumPy array
inputs = []

for i in column_names:
    inputs.append(file[i].values)



#List of unique values in each columns
uniques = []

print(inputs[1])

#skips row 0, because it is header information - np.unique() is used to give the sorted array of unique values within the columnm and the index of the first of that unique value
for i in inputs:
    u, indices = np.unique(i[1:], return_index=True)
    uniques.append(u)
# uniques will have two values = location, date -> this step does not sort the values. 


#Assigns unique number to each unique column value
# for each unique location, find a row where that location appears and assign label equal to its index - each scats site (location) is maped to an ID 
x1 = np.zeros(len(inputs[0][1:]))

for i, n in enumerate(uniques[0]): # gives you an index of the unique value based on its index in uniques. 
    indices = np.where(n == inputs[0][1:])[0]
    x1[indices] = i


# instead of storing each date, we should store day of the week, as we are making a prediction of traffic based on day of the week, not day of the month. 
# for each unique date, we convert the date to an integer (0-6) and assign the integers a day-of-week value
x2 = np.zeros(len(inputs[1][1:]))

for i, n in enumerate(uniques[1]):
    indices = np.where(n == inputs[1][1:])[0]
    x2[indices] = pd.Timestamp(n).dayofweek


#Combine x1 and x2 into final input matrix
inputs = np.array([x1, x2]).T


# for testing purposes, we print only the first 10 rows. 
for i in inputs:
    print("\nFirst 10 rows of inputs [location, day_of_week]:")
    print(inputs[:10])
