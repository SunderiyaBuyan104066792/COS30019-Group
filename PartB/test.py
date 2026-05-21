import os
import math
import warnings
import numpy as np
import pandas as pd
from data.preprocessing import process_data
from keras.models import load_model
import sklearn.metrics as metrics
import matplotlib as mpl
import matplotlib.pyplot as plt
warnings.filterwarnings("ignore")

LAGS = 4


def MAPE(y_true, y_pred):
    """Mean Absolute Percentage Error
    Filters out zero values to avoid division by zero.

    # Arguments
        y_true: List/ndarray, true data.
        y_pred: List/ndarray, predicted data.
    # Returns
        mape: Double, mean absolute percentage error.
    """
    y_pred = [y_pred[i] for i in range(len(y_true)) if y_true[i] > 0]
    y_true = [x for x in y_true if x > 0]

    num = len(y_true)
    sums = sum(abs(y_true[i] - y_pred[i]) / y_true[i] for i in range(num))

    return sums * (100 / num)


def eva_regress(y_true, y_pred):
    """Evaluation
    Evaluate the predicted result.

    # Arguments
        y_true: List/ndarray, true data.
        y_pred: List/ndarray, predicted data.
    """
    mape = MAPE(y_true, y_pred)
    vs   = metrics.explained_variance_score(y_true, y_pred)
    mae  = metrics.mean_absolute_error(y_true, y_pred)
    mse  = metrics.mean_squared_error(y_true, y_pred)
    r2   = metrics.r2_score(y_true, y_pred)

    print('  explained_variance_score: %f' % vs)
    print('  mape: %f%%' % mape)
    print('  mae: %f' % mae)
    print('  mse: %f' % mse)
    print('  rmse: %f' % math.sqrt(mse))
    print('  r2: %f' % r2)


def plot_results(y_true, y_preds, names, site, direction):
    """Plot
    Plot the true data and predicted data for one day (96 intervals).

    # Arguments
        y_true: ndarray, true data.
        y_preds: List of ndarray, predicted data per model.
        names: List, model names.
        site: String, SCATS site number.
        direction: String, direction name.
    """
    # One day = 96 intervals of 15 minutes
    d = '2006-10-25 00:00'
    x = pd.date_range(d, periods=96, freq='15min')

    fig = plt.figure()
    ax = fig.add_subplot(111)

    ax.plot(x, y_true[:96], label='True Data')
    for name, y_pred in zip(names, y_preds):
        ax.plot(x, y_pred[:96], label=name)

    plt.legend()
    plt.grid(True)
    plt.xlabel('Time of Day')
    plt.ylabel('Flow (vehicles/15min)')
    plt.title(f'Site {site} | {direction}')

    date_format = mpl.dates.DateFormatter("%H:%M")
    ax.xaxis.set_major_formatter(date_format)
    fig.autofmt_xdate()

    plt.tight_layout()
    plt.show()


def evaluate_all():
    """Evaluate all trained models and print average metrics."""
    results = {'lstm': [], 'gru': []}

    for site in os.listdir('trained_models'):
        site_path = os.path.join('trained_models', site)
        if not os.path.isdir(site_path):
            continue

        for direction in os.listdir(site_path):
            train_path = f'data/SCATS_Data/{site}/{direction}/train.csv'
            test_path = f'data/SCATS_Data/{site}/{direction}/test.csv'

            if not os.path.exists(train_path):
                continue

            _, _, X_test, y_test, scaler = process_data(train_path, test_path, LAGS)
            y_test_real = scaler.inverse_transform(y_test.reshape(-1, 1)).reshape(1, -1)[0]
            X = np.reshape(X_test, (X_test.shape[0], X_test.shape[1], 1))

            for model_name in ['lstm', 'gru']:
                model_path = f'trained_models/{site}/{direction}/{model_name}.h5'
                if not os.path.exists(model_path):
                    continue

                model = load_model(model_path, compile=False)
                predicted = model.predict(X, verbose=0)
                predicted = scaler.inverse_transform(predicted.reshape(-1, 1)).reshape(1, -1)[0]

                mse = metrics.mean_squared_error(y_test_real, predicted)
                results[model_name].append({
                    'mae': metrics.mean_absolute_error(y_test_real, predicted),
                    'rmse': math.sqrt(mse),
                    'mape': MAPE(y_test_real, predicted),
                    'r2': metrics.r2_score(y_test_real, predicted),
                })

    print('\n=== Average metrics across all directions ===')
    for model_name, res in results.items():
        if not res:
            continue
        df = pd.DataFrame(res)
        print(f'\n{model_name.upper()} ({len(res)} directions):')
        print(f' MAE:  {df["mae"].mean():.4f}')
        print(f' RMSE: {df["rmse"].mean():.4f}')
        print(f' MAPE: {df["mape"].mean():.4f}%')
        print(f'R2:   {df["r2"].mean():.4f}')


def main():
    # Evaluate on one direction for example
    site = '4321'
    direction = 'HIGH_ST_NE_OF_HARP_ST'

    train_path = f'data/SCATS_Data/{site}/{direction}/train.csv'
    test_path = f'data/SCATS_Data/{site}/{direction}/test.csv'

    _, _, X_test, y_test, scaler = process_data(train_path, test_path, LAGS)
    y_test = scaler.inverse_transform(y_test.reshape(-1, 1)).reshape(1, -1)[0]

    models = [
        load_model(f'trained_models/{site}/{direction}/lstm.h5', compile=False),
        load_model(f'trained_models/{site}/{direction}/gru.h5', compile=False)
    ]
    names = ['LSTM', 'GRU']

    y_preds = []
    for name, model in zip(names, models):
        X = np.reshape(X_test, (X_test.shape[0], X_test.shape[1], 1))
        predicted = model.predict(X, verbose=0)
        predicted = scaler.inverse_transform(predicted.reshape(-1, 1)).reshape(1, -1)[0]
        y_preds.append(predicted)
        print(name)
        eva_regress(y_test, predicted)

    plot_results(y_test, y_preds, names, site, direction)

    # Average metrics across all directions, close the graph window to see output. May time few minutes.
    evaluate_all()


if __name__ == '__main__':
    main()