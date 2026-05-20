import sys
import warnings
import argparse
import numpy as np
import pandas as pd



# First we have to load the dataset:
# we use pd.read_excel documentation to understand sheet_name = tab, and engine


#Read excel file
file = pd.read_excel('Scats Data October 2006.xls', sheet_name=1, engine='calamine')


#Select columns to be used as input
input_idxs = [1, 9]
columns_names = ['LOCATION', 'DATE']

#Get column names- from column header
column_names = file.columns[input_idxs]


# to build our y_train data
# we need to create something to grab the row names for the time intervals
labels = []

for row in range(1, file.shape[0]): # our file shape is (rows=4193, columns=106)

    # using iloc =https://www.geeksforgeeks.org/pandas/python-extracting-rows-using-pandas-iloc/
    # parameters = index position of rows in int or list of int
    # we are usng it to return the row as one value per column
    # slice at position 10 (from here these are the time intervals)
    # temp holds the 96 vehicle counts for that site, date
    # to_numpy() converts to 1D array
    temp = file.iloc[row][10:].to_numpy() 

    # labels is a list of the rows, each with 96 values
    labels.append(temp)

# convert the list into one 2D numpy array shape(4193, 96)
labels = np.array(labels)

# flattern to 1D - so later preprocessing can have the same positions. 
labels = labels.flatten()

# we also need to sort it specifically for keras 
# wraps the label in [] then converts it back to numpy array
# this creates a 2D array with one element per column
labels = np.array([[label] for label in labels])

# labels is now a (num of elements, 1) 2D array



# same with the values



#List of column values- iterates over the selected column names and appends the values into a NumPy array
inputs = []

for i in column_names:
    inputs.append(file[i].values)



#List of unique values in each columns
uniques = []


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


    for time_int in range(96): # 0 -> 96


        out_idx = row_idx * 96 + time_int # position in expanded array - slot is time interval

        x1_expanded[out_idx] = x1[row_idx]
        x2_expanded[out_idx] = x2[row_idx]
        x3[out_idx] = time_int

# Combine into final input matrix
inputs = np.array([x1_expanded, x2_expanded, x3]).T


np.savetxt('train_x.csv', inputs)
np.savetxt('train_y.csv', labels)



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




#