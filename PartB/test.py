import os
import sys
import math
import warnings
import argparse
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
    results = {'lstm': [], 'gru': [], 'custom': []}

    for site in os.listdir('trained_models'):
        site_path = os.path.join('trained_models', site)
        if not os.path.isdir(site_path):
            continue

        for direction in os.listdir(site_path):
            train_path = f'data/SCATS_Data/{site}/{direction}/train.csv'
            test_path = f'data/SCATS_Data/{site}/{direction}/test.csv'

            if not os.path.exists(train_path):
                continue

            _, _, _, Xf, Xl, y_c, scaler = process_data(train_path, test_path, LAGS)
            y_eval = scaler.inverse_transform(y_c.reshape(-1, 1)).reshape(1, -1)[0]

            for model_name in ['lstm', 'gru', 'custom']:
                model_path = f'trained_models/{site}/{direction}/{model_name}.h5'
                if not os.path.exists(model_path):
                    continue

                model = load_model(model_path, compile=False)
                predicted = model.predict([Xf, Xl], verbose=0)
                predicted = scaler.inverse_transform(predicted.reshape(-1, 1)).reshape(1, -1)[0]

                mse = metrics.mean_squared_error(y_eval, predicted)
                results[model_name].append({
                    'mae': metrics.mean_absolute_error(y_eval, predicted),
                    'rmse': math.sqrt(mse),
                    'mape': MAPE(y_eval, predicted),
                    'r2': metrics.r2_score(y_eval, predicted),
                })

    print('\n=== Average metrics across all directions ===')
    for model_name, res in results.items():
        if not res:
            continue
        df = pd.DataFrame(res)
        print(f'\n{model_name.upper()} ({len(res)} directions):')
        print(f' MAE:{df["mae"].mean():.4f}')
        print(f' RMSE: {df["rmse"].mean():.4f}')
        print(f' MAPE: {df["mape"].mean():.4f}%')
        print(f'R2:   {df["r2"].mean():.4f}')


def pick_direction(site):

    site_path  = os.path.join('data/SCATS_Data', str(site))
    directions = sorted([
        d for d in os.listdir(site_path)
        if os.path.isdir(os.path.join(site_path, d))
    ])

    if not directions:
        print(f'No directions found for site {site}.')
        sys.exit(1)

    print(f'\nAvailable directions for site {site}:')
    for i, d in enumerate(directions, 1):
        print(f'  {i}. {d}')

    while True:
        try:
            choice = int(input('\nEnter number: '))
            if 1 <= choice <= len(directions):
                return directions[choice - 1]
            print(f'Please choose a number between 1 and {len(directions)}.')
        except ValueError:
            print('Invalid input, choose from above options')


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--scats',
        type=int,
        default=None,
        help='SCATS site number e.g. 970 (omit to enter interactively)')
    args = parser.parse_args()

    if args.scats is not None:
        site = args.scats
    else:
        while True:
            try:
                site = int(input('\nEnter SCATS site number: '))
                if os.path.isdir(os.path.join('data/SCATS_Data', str(site))):
                    break
                print(f'Site {site} not found in data/SCATS_Data/.')
            except ValueError:
                print('Invalid input, enter a number.')

    direction = pick_direction(site)

    train_path = f'data/SCATS_Data/{site}/{direction}/train.csv'
    test_path  = f'data/SCATS_Data/{site}/{direction}/test.csv'

    _, _, _, Xf, Xl, y_c, scaler = process_data(train_path, test_path, LAGS)
    y_test = scaler.inverse_transform(y_c.reshape(-1, 1)).reshape(1, -1)[0]

    y_preds = []
    names   = []

    for model_name in ['lstm', 'gru', 'custom']:
        model_path = f'trained_models/{site}/{direction}/{model_name}.h5'
        if not os.path.exists(model_path):
            print(f'\n{model_name.upper()}: model not found, skipping.')
            continue

        model     = load_model(model_path, compile=False)
        predicted = scaler.inverse_transform(
            model.predict([Xf, Xl], verbose=0).reshape(-1, 1)).reshape(1, -1)[0]
        y_preds.append(predicted)
        names.append(model_name.upper())
        print(f'\n{model_name.upper()}:')
        eva_regress(y_test, predicted)

    if y_preds:
        plot_results(y_test, y_preds, names, site, direction)
        print('\nClose the graph window to see average metrics...')

    evaluate_all()

if __name__ == '__main__':
    main(sys.argv)
