"""
Saves traffic flow predictions for all directions.
Looks up from the file instead of running models per scat and per direction everytime.

"""
import os
import numpy as np
import pandas as pd
from keras.models import load_model
from sklearn.preprocessing import MinMaxScaler

def make_features(df):
    dt = pd.to_datetime(df['datetime'])
    x2 = dt.dt.dayofweek.values / 6.0
    x3 = (dt.dt.hour * 4 + dt.dt.minute // 15).values / 95.0
    return np.array([x2, x3]).T.astype('float32')


def prepare_direction(site, direction):
    train_path = os.path.join('data/SCATS_Data', site, direction, 'train.csv')
    test_path = os.path.join('data/SCATS_Data', site, direction, 'test.csv')

    if not os.path.exists(train_path):
        return

    # Load all
    df_train = pd.read_csv(train_path).fillna(0)
    df_test  = pd.read_csv(test_path).fillna(0)
    df = pd.concat([df_train, df_test], ignore_index=True)
    df['datetime'] = pd.to_datetime(df['datetime'])
    df = df.sort_values('datetime').reset_index(drop=True)

    # Fit scaler on training data only
    scaler = MinMaxScaler(feature_range=(0, 1))
    scaler.fit(df_train[['volume']])

    # Scale all
    volumes_scaled = scaler.transform(df[['volume']]).flatten()

    feat = make_features(df)
    datetimes = df['datetime'].iloc[4:].values

    Xf = np.array([feat[i] for i in range(4, len(volumes_scaled))], dtype='float32')
    Xl = np.array([volumes_scaled[i - 4: i] for i in range(4, len(volumes_scaled))], dtype='float32').reshape(-1, 4, 1)

    # Predict with models
    result = pd.DataFrame({'datetime': datetimes})

    for model_name in ['lstm', 'gru', 'custom']:
        model_path = os.path.join('trained_models', site, direction, model_name + '.h5')
        if not os.path.exists(model_path):
            continue

        model  = load_model(model_path, compile=False)
        predicted = model.predict([Xf, Xl], verbose=0)
        predicted = scaler.inverse_transform(predicted).flatten()
        predicted = np.maximum(predicted, 0)

        result[model_name] = predicted

    # Save predictions
    out_dir = os.path.join('predictions', site, direction)
    os.makedirs(out_dir, exist_ok=True)
    result.to_csv(os.path.join(out_dir, 'predictions.csv'), index=False)

def save_all():
    total = 0

    for site in os.listdir('trained_models'):
        site_path = os.path.join('trained_models', site)
        if not os.path.isdir(site_path):
            continue

        for direction in os.listdir(site_path):
            prepare_direction(site, direction)
            print(f' {site} | {direction}')
            total += 1

    print(f'\nDone.')


if __name__ == '__main__':
    save_all()