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
    Create (X, y) pairs from a 1D normalized time series.
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

def process_data(file_path, lag=4, train_ratio=0.8, scats_number=None):
    """
    Process data for a single SCATS site (or the whole file if scats_number
    is None).  Returns train/test sequence splits plus the fitted scaler.

    Returns
    -------
    X_train, y_train, X_test, y_test : ndarrays
    scaler                           : fitted MinMaxScaler
    """
    
    file = read_data(file_path)
    
    if scats_number is not None:
        file = file[file['SCATS Number'] == scats_number].reset_index(drop=True)
        print(f"Filtered to SCATS site: {scats_number} ({len(file)} rows)")
    
    flow_cols   = ['V%02d' % i for i in range(96)]
    flow        = file[flow_cols].values.flatten().astype(float)
    
    scaler      = MinMaxScaler()
    flow_scaled = scaler.fit_transform(flow.reshape(-1, 1)).flatten()
    
    X_seq, y_seq = create_sequences(flow_scaled, lag)
    
    s = int(len(X_seq) * train_ratio)
    X_train, y_train = X_seq[:s], y_seq[:s]
    X_test,  y_test  = X_seq[s:], y_seq[s:]
    
    print(f"Total samples:   {len(flow_scaled)}")
    print(f"LSTM/GRU train:  {len(X_train)}  |  test: {len(X_test)}  |  shape: {X_train.shape}")
    
    return X_train, y_train, X_test, y_test, scaler


def process_data_multi(file_path, scats_list, lag=4, train_ratio=0.8):
    """
    Pool traffic-flow data from multiple SCATS sites and return a single
    train/test split.

    Parameters
    ----------
    file_path   : str   path to the SCATS Excel file
    scats_list  : list  SCATS site numbers to include (e.g. [970, 2000, 2200])
    lag         : int   sequence length (default 12 → 3 hours of 15-min slots)
    train_ratio : float fraction of sequences used for training (default 0.8)

    Returns
    -------
    X_train : ndarray, shape (N_train, lag)
    y_train : ndarray, shape (N_train,)
    X_test  : ndarray, shape (N_test,  lag)
    y_test  : ndarray, shape (N_test,)
    scaler  : fitted MinMaxScaler  (inverse-transform predictions with this)
    """
    
    file      = read_data(file_path)
    flow_cols = ['V%02d' % i for i in range(96)]
    
    # Collect raw flow from each site in sorted order
    segments = []
    for scats_id in sorted(scats_list):
        site_rows = file[file['SCATS Number'] == scats_id]
        if site_rows.empty:
            print(f"  [process_data_multi] WARNING: SCATS {scats_id} not found — skipped.")
            continue
        seg = site_rows[flow_cols].values.flatten().astype(float)
        segments.append(seg)
        print(f"  SCATS {scats_id}: {len(site_rows)} rows → {len(seg)} flow values")
    
    if not segments:
        raise ValueError(f"No valid data found for sites: {scats_list}")
    
    # Concatenate all sites into one long series
    flow_all = np.concatenate(segments)
    
    # Fit a single scaler on the whole cluster's data
    scaler      = MinMaxScaler()
    flow_scaled = scaler.fit_transform(flow_all.reshape(-1, 1)).flatten()
    
    X_seq, y_seq = create_sequences(flow_scaled, lag)
    
    s = int(len(X_seq) * train_ratio)
    X_train, y_train = X_seq[:s], y_seq[:s]
    X_test,  y_test  = X_seq[s:], y_seq[s:]
    
    print(f"\n  Cluster total samples : {len(flow_scaled)}")
    print(f"  Train sequences       : {len(X_train)}")
    print(f"  Test  sequences       : {len(X_test)}")
    print(f"  Input shape           : {X_train.shape}")
    
    return X_train, y_train, X_test, y_test, scaler

if __name__ == '__main__':
    X_train_seq, y_train_seq, X_test_seq, y_test_seq, scaler = process_data('Scats_Data_Oct_2006.xls', lag = 4)
    
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
