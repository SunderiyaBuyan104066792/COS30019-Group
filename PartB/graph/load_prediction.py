"""
Looks up saved traffic flow predictions for a given site, direction, datetime and model.
"""
import os
import pandas as pd

# Cache used to avoid repeated CSV reads
cache = {}

def load_predictions(site, direction):
    key = (str(site), direction)
    if key not in cache:
        path = os.path.join('../predictions', str(site), direction, 'predictions.csv')
        if not os.path.exists(path):
            return None
        df = pd.read_csv(path)
        df['datetime'] = pd.to_datetime(df['datetime'])
        df = df.set_index('datetime')
        cache[key] = df
    return cache[key]


def get_flow(site, direction, query_time, model_name='lstm'):
    df = load_predictions(site, direction)
    if df is None or model_name not in df.columns:
        return None

    idx = df.index.get_indexer([query_time], method='nearest')[0]
    return float(df.iloc[idx][model_name])
