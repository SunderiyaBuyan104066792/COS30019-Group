import os
import warnings
import numpy as np
import pandas as pd
from sklearn.preprocessing import MinMaxScaler
from keras.models import load_model
warnings.filterwarnings('ignore', message='X does not have valid feature names')

LAGS = 8

_model_cache = {}
_scaler_cache = {}
_data_cache = {}


def _get_scaler(site, direction):
    key = (site, direction)
    if key not in _scaler_cache:
        train_path = os.path.join('data/SCATS_Data', str(site), direction, 'train.csv')
        df = pd.read_csv(train_path).fillna(0)
        scaler = MinMaxScaler(feature_range=(0, 1))
        scaler.fit(df[['volume']])
        _scaler_cache[key] = scaler
    return _scaler_cache[key]


def _get_model(site, direction, model_name):
    key = (site, direction, model_name)
    if key not in _model_cache:
        path = os.path.join('trained_models', str(site), direction, model_name + '.h5')
        if not os.path.exists(path):
            return None
        _model_cache[key] = load_model(path, compile=False)
    return _model_cache[key]


def _get_data(site, direction):
    key = (site, direction)
    if key not in _data_cache:
        train_path = os.path.join('data/SCATS_Data', str(site), direction, 'train.csv')
        test_path = os.path.join('data/SCATS_Data', str(site), direction, 'test.csv')
        df = pd.concat([
            pd.read_csv(train_path).fillna(0),
            pd.read_csv(test_path).fillna(0)
        ], ignore_index=True)
        df['datetime'] = pd.to_datetime(df['datetime'])
        df = df.sort_values('datetime').reset_index(drop=True)
        _data_cache[key] = df
    return _data_cache[key]


def predict_november(site, query_time, model_name='lstm'):
    """Predict traffic flow for a November 2006 datetime at a SCATS site."""
    site_path = os.path.join('data/SCATS_Data', str(site))
    if not os.path.exists(site_path):
        return None

    directions = [
        d for d in os.listdir(site_path)
        if os.path.isdir(os.path.join(site_path, d))
    ]
    if not directions:
        return None

    direction = directions[0]
    return _predict(site, direction, query_time, model_name)


def predict_for_direction(site, direction, query_time, model_name='lstm'):
    """Predict traffic flow for a specific site, direction, and November datetime."""
    return _predict(site, direction, query_time, model_name)


def _predict(site, direction, query_time, model_name):
    model = _get_model(site, direction, model_name)
    scaler = _get_scaler(site, direction)
    df = _get_data(site, direction)

    if model is None:
        return None

    target_time = query_time.time()
    target_dayofweek = query_time.weekday()

    df['time_only'] = df['datetime'].dt.time
    df['dow'] = df['datetime'].dt.dayofweek
    matching = df[
        (df['time_only'] == target_time) &
        (df['dow'] == target_dayofweek)
    ].sort_values('datetime')

    if len(matching) >= 1:
        anchor_idx = matching.index[-1]
        start_idx = max(0, anchor_idx - LAGS + 1)
        lag_rows = df.iloc[start_idx: anchor_idx + 1]
    else:
        lag_rows = df.tail(LAGS)

    if len(lag_rows) < LAGS:
        return float(df['volume'].mean())

    volumes = lag_rows['volume'].values[-LAGS:].reshape(-1, 1)
    Xl = scaler.transform(volumes).reshape(1, LAGS, 1).astype('float32')

    day_norm = query_time.weekday() / 6.0
    slot = query_time.hour * 4 + query_time.minute // 15
    time_norm = slot / 95.0
    Xf = np.array([[day_norm, time_norm]], dtype='float32')

    pred = model.predict([Xf, Xl], verbose=0)
    volume = scaler.inverse_transform(pred)[0][0]
    return max(0.0, float(volume))


if __name__ == '__main__':
    from datetime import datetime

    site = 970
    direction = 'WARRIGAL_RD_N_of_HIGH_STREET_RD'

    nov_times = [
        datetime(2006, 11, 6, 0, 0),
        datetime(2006, 11, 6, 8, 0),
        datetime(2006, 11, 6, 12, 0),
        datetime(2006, 11, 6, 17, 0),
        datetime(2006, 11, 11, 8, 0),
    ]

    print(f'November predictions — site {site} | {direction}:')
    for t in nov_times:
        flow = predict_for_direction(site, direction, t, 'lstm')
        print(f'  {t.strftime("%a %Y-%m-%d %H:%M")}  ->  {flow:.1f} veh/15min')
