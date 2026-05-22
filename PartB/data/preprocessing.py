"""
Processing the data
"""
import numpy as np
import pandas as pd
from sklearn.preprocessing import MinMaxScaler


def process_data(train, test, lags):
    """Process data
    Reshape and split train/test data.

    # Arguments
        train: String, name of .csv train file.
        test: String, name of .csv test file.
        lags: integer, time lag.
    # Returns
        X_train: ndarray.
        y_train: ndarray.
        X_test: ndarray.
        y_test: ndarray.
        scaler: MinMaxScaler.
    """
    attr = 'volume'
    df1 = pd.read_csv(train, encoding='utf-8').fillna(0)
    df2 = pd.read_csv(test, encoding='utf-8').fillna(0)

    scaler = MinMaxScaler(feature_range=(0, 1)).fit(df1[attr].values.reshape(-1, 1))
    flow1 = scaler.transform(df1[attr].values.reshape(-1, 1)).reshape(1, -1)[0]
    flow2 = scaler.transform(df2[attr].values.reshape(-1, 1)).reshape(1, -1)[0]

    train, test = [], []
    for i in range(lags, len(flow1)):
        train.append(flow1[i - lags: i + 1])
    for i in range(lags, len(flow2)):
        test.append(flow2[i - lags: i + 1])

    train = np.array(train)
    test  = np.array(test)
    np.random.shuffle(train)

    X_train = train[:, :-1]
    y_train = train[:, -1]
    X_test  = test[:, :-1]
    y_test  = test[:, -1]

    return X_train, y_train, X_test, y_test, scaler




def process_data_custom(train, test, lags):
    """
    Preprocessing for the DFF and DCN model:

    - adpated to fit the expectations od train.py and test.py


    Built the same way I built my original preprocessing - deriving the input
        features explicitly - but adapted to the per-direction setup so the output matches what train.py / test.py now expects
  
    
        # Returns
            Xf_train: ndarray(number, 2), [day-of-week, time-of-day] features.
            Xl_train: ndarray(number, lags, 1), lag-window input.
            y_train:  ndarray(number, ), target volume.
            Xf_test, Xl_test, y_test: same for the test set.
            scaler: MinMaxScaler, fit on train volume (same as process_data).

    """

    #loads the train  and test csv for one direction
    attr = 'volume'
    df1 = pd.read_csv(train, encoding='utf-8').fillna(0)
    df2 = pd.read_csv(test, encoding='utf-8').fillna(0)
 
    # Same volume scaling as process_data- MinMaxScaler(feature_range)- scaler is fit of df1(train) 
    scaler = MinMaxScaler(feature_range=(0, 1)).fit(df1[attr].values.reshape(-1, 1))

    
    flow1 = scaler.transform(df1[attr].values.reshape(-1, 1)).reshape(1, -1)[0] #train vlumens
    flow2 = scaler.transform(df2[attr].values.reshape(-1, 1)).reshape(1, -1)[0] #test volumes
 

    def make_features(df):
        dt = pd.to_datetime(df['datetime'])
        x2 = dt.dt.dayofweek.values / 6.0
        x3 = (dt.dt.hour * 4 + dt.dt.minute // 15).values / 95.0
        # stack the two feature columns, one row per timestamp (like inputs.T )
        return np.array([x2, x3]).T.astype('float32')
 
    feat1 = make_features(df1)
    feat2 = make_features(df2)
 
    # Build the lag windows + targets, same windowing as process_data: for each
    # row i the model sees the `lags` volumes before it (Xl) plus that row's
    # day/time (Xf), and predicts the volume at i (y).
    def build(flow, feat):
        Xf, Xl, y = [], [], []
        for i in range(lags, len(flow)):
            Xl.append(flow[i - lags: i]) # the `lags` preceding volumes
            y.append(flow[i]) # the target volume
            Xf.append(feat[i]) # day/time of the predicted row
        return (np.array(Xf, dtype='float32'),
                np.array(Xl, dtype='float32'),
                np.array(y,  dtype='float32'))
 
    Xf_train, Xl_train, y_train = build(flow1, feat1)
    Xf_test,  Xl_test,  y_test = build(flow2, feat2)
 
    # Shuffle the training set (same as process_data), keeping the lag windows,
    # features and targets aligned to each other.
    idx = np.arange(len(y_train))
    np.random.shuffle(idx)

    Xf_train, Xl_train, y_train = Xf_train[idx], Xl_train[idx], y_train[idx]
 
    # Conv1D needs a channel dimension on the lag window.
    Xl_train = Xl_train.reshape(Xl_train.shape[0], Xl_train.shape[1], 1)
    Xl_test = Xl_test.reshape(Xl_test.shape[0], Xl_test.shape[1],  1)
 
    return Xf_train, Xl_train, y_train, Xf_test, Xl_test, y_test, scaler