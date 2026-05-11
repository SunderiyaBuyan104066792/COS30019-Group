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

# right now we have one value per day, per site- now we need to consider one value per day, per time, per site


# we also need to look at the time interval (not a column)
num_rows = len(inputs[0][1:]) # current number of rows after x1 and x2

x1_expanded = np.zeros(num_rows * 96) # each need a time interval (96 values)
x2_expanded = np.zeros(num_rows * 96)
x3 = np.zeros(num_rows * 96)

for row_idx in range(num_rows): # 0 -> num_rows
    for slot in range(96): # 0 -> 96
        out_idx = row_idx * 96 + slot # position in expanded array - slot is time interval

        x1_expanded[out_idx] = x1[row_idx]
        x2_expanded[out_idx] = x2[row_idx]
        x3[out_idx] = slot

# Combine into final input matrix
inputs = np.array([x1_expanded, x2_expanded, x3]).T

# for testing purposes, we print only the first 10 rows. 
for i in inputs:
    print("\nFirst 10 rows of inputs [location, day_of_week, time_slot]:")
    print(inputs[:10])
    print(f"\nFinal shape: {inputs.shape}")



# further preprocessing:










# What inputs does a deep forward feed require:
# location
# time-slot (every 15 mins)
# day of the week (0-6)
#vehicle count- not done in preprocessing


# past vehicle counts- only necessary for DFF


# post processing:
# the site types are hard to consider as they do not effect the intersections, which is all of the excel data.
# we could implement it either as a penalty to the calculated travel time (eg. school zones will only allow speed of 40km/h), or assign adjacent scats data the characteristic (I'm not really sure how that could change it too much)

