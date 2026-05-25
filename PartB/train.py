import os
import numpy as np
import pandas as pd
from data.preprocessing import process_data
from models import get_lstm, get_gru, get_custom
from keras.callbacks import EarlyStopping


def train_model(model, X_train, y_train, name, config):
    """Train a single model.

    # Arguments
        model: Model, NN model to train.
        X_train: ndarray(number, lags, 1), Input data for train.
        y_train: ndarray(number, ), result data for train.
        name: String, path to save model.
        config: Dict, batch and epoch settings.
    """
    model.compile(loss='mse', optimizer='adam', metrics=['mape']) #used adam is bit better

    hist = model.fit(
        X_train, y_train,
        batch_size=config['batch'],
        epochs=config['epochs'],
        validation_split=0.05,
        callbacks=[EarlyStopping(monitor='val_loss', patience=15,restore_best_weights=True, verbose=0)],
        verbose=0
    )

    os.makedirs(os.path.dirname(name), exist_ok=True)
    model.save(name + '.h5')

    df = pd.DataFrame.from_dict(hist.history)
    df.to_csv(name + '_loss.csv', encoding='utf-8', index=False)


def train_all(model_name, test_run=False):
    config = {'batch': 32, 'epochs': 100}

    for site in os.listdir('data/SCATS_Data'):
        site_path = os.path.join('data/SCATS_Data', site)
        if not os.path.isdir(site_path):
            continue

        for direction in os.listdir(site_path):
            direction_path = os.path.join(site_path, direction)
            train_path = os.path.join(direction_path, 'train.csv')
            test_path  = os.path.join(direction_path, 'test.csv')

            if not os.path.exists(train_path):
                continue

            # all models take [features, lags] dual input
            Xf, Xl, y_train, _, _, _, _ = process_data(train_path, test_path, 4)
            X_train = [Xf, Xl]

            if model_name == 'lstm':
                m = get_lstm(4)
            elif model_name == 'gru':
                m = get_gru(4)
            elif model_name == 'custom':
                m = get_custom(4)

            save_path = os.path.join('trained_models', site, direction, model_name)
            train_model(m, X_train, y_train, save_path, config)
            print(f'Trained {model_name} | {site} | {direction}')

        if test_run:
            print('Test run complete for the test site')
            break


def main():
    # Set test_run=True to train one site only, False for all sites
    test_run = False

    for model_name in ['lstm', 'gru', 'custom']:
        print(f'\nTraining {model_name.upper()}...')
        train_all(model_name, test_run=test_run)
    print('\nDone.')


if __name__ == '__main__':
    main()
