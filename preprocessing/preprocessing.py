import numpy as np
import math
import h5py
import os
import pandas as pd
import sys
from sklearn.preprocessing import MinMaxScaler



# First we have to load the dataset:
def read_data(file_path):
    # we use pd.read_excel documentation to understand sheet_name = tab, and engine
    #Read excel file
    file = pd.read_excel(file_path, sheet_name='Data', engine='xlrd', header=1)
    file = file.iloc[1:]  # skip metadata row
    file = file.reset_index(drop=True)

    return file

def create_sequences(data, lag):
    """
    Create (X, y) pairs from a 1D normalised time series.
    Used by LSTM and GRU — they need sequences of past `lag` steps.

    Returns
    -------
    X : ndarray, shape (N, lag)
    y : ndarray, shape (N,)
    """
    X, y = [], []
    for i in range(len(data) - lag):
        X.append(data[i : i + lag])
        y.append(data[i + lag])
    return np.array(X), np.array(y)

def process_data(file_path, lag=12, train_ratio=0.8):
    file = read_data(file_path)
    
    # Extract locations and dates columns
    locations = file['Location'].values
    dates = file['Date'].values
    
    # Map each unique location to a numeric ID
    unique_locations = np.unique(locations)
    x1 = np.zeros(len(locations))
    for i, n in enumerate(unique_locations): # gives you an index of the unique value based on its index in uniques. 
        indices = np.where(n == locations)[0]
        x1[indices] = i
    
    
    # instead of storing each date, we should store day of the week, as we are making a prediction of traffic based on day of the week, not day of the month. 
    # for each unique date, we convert the date to an integer (0-6) and assign the integers a day-of-week value
    unique_dates = np.unique(dates)
    x2 = np.zeros(len(dates))
    for i, n in enumerate(unique_dates):
        indices = np.where(n == dates)[0]
        x2[indices] = pd.Timestamp(n).dayofweek
    
    # right now we have one value per day, per site- now we need to consider one value per day, per time, per site
    
    
    # we also need to look at the time interval (not a column)
    num_rows = len(locations) # current number of rows after x1 and x2
    x1_expanded = np.zeros(num_rows * 96) # each need a time interval (96 values)
    x2_expanded = np.zeros(num_rows * 96)
    x3 = np.zeros(num_rows * 96)
    
    for row_idx in range(num_rows): # 0 -> num_rows
        for slot in range(96): # 0 -> 96
            out_idx = row_idx * 96 + slot # position in expanded array - slot is time interval
            x1_expanded[out_idx] = x1[row_idx]
            x2_expanded[out_idx] = x2[row_idx]
            x3[out_idx] = slot

    # Extract traffic flow values V00-V95 as target y
    flow_cols = ['V%02d' % i for i in range(96)]
    flow = file[flow_cols].values.flatten().astype(float)
    scaler = MinMaxScaler()
    flow_scaled = scaler.fit_transform(flow.reshape(-1, 1)).flatten()

    def split_data(X, y):
        s = int(len(X) * train_ratio)
        return X[:s], y[:s], X[s:], y[s:]
    
    # Build sequences for LSTM / GRU
    X_seq, y_seq = create_sequences(flow_scaled, lag)

    X_train_seq, y_train_seq, X_test_seq, y_test_seq = split_data(X_seq, y_seq)
    print(f"Total samples:   {len(flow_scaled)}")
    print(f"LSTM/GRU train:  {len(X_train_seq)}  |  test: {len(X_test_seq)}  |  shape: {X_train_seq.shape}")
    
    # Combine into final input matrix
    inputs = np.array([x1_expanded, x2_expanded, x3]).T

    return X_train_seq, y_train_seq, X_test_seq, y_test_seq, scaler

if __name__ == '__main__':
    X_train_seq, y_train_seq, X_test_seq, y_test_seq, scaler = process_data('Scats_Data_Oct_2006.xls', lag = 12)
    
    # for testing purposes, we print only the first 10 rows. 
    print(f"X_train shape: {X_train_seq.shape}")
    print(f"y_train shape: {y_train_seq.shape}")
    print(f"X_test  shape: {X_test_seq.shape}")
    print(f"y_test  shape: {y_test_seq.shape}")
    


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

