import math
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl

from keras.models import load_model
from sklearn.metrics import (
    mean_absolute_error,
    mean_squared_error,
    mean_absolute_percentage_error,
    r2_score, 
    explained_variance_score
)
from data.process_data import process_data, read_data

# Settings
DATA_FILE = 'data/Scats_Data_Oct_2006.xls'


def evaluate(y_true, y_pred, model_name='LSTM'):
    """
    Metrics explained:
        MAE:  average absolute error (in number of cars)
        MSE:  average squared error (penalizes big errors more)
        RMSE: square root of MSE, same unit as traffic flow
        MAPE: error as a percentage — easy to understand
        R2:   how well predictions fit actual (1.0 = perfect)
        EVS:  explained variance score (1.0 = perfect)
    """
    
    mae = mean_absolute_error(y_true, y_pred)
    mse = mean_squared_error(y_true, y_pred)
    rmse = math.sqrt(mse)
    mape = mean_absolute_percentage_error(y_true, y_pred) * 100
    r2 = r2_score(y_true, y_pred)
    evs = explained_variance_score(y_true, y_pred)
    
    print(f"\n--- {model_name} Evaluation ---")
    print(f"  MAE:   {mae:.4f}  (avg error in cars/15min)")
    print(f"  MSE:   {mse:.4f}")
    print(f"  RMSE:  {rmse:.4f}")
    print(f"  MAPE:  {mape:.2f}%  (avg % error)")
    print(f"  R2:    {r2:.4f}   (1.0 = perfect)")
    print(f"  EVS:   {evs:.4f}  (1.0 = perfect)")
    
    return {'MAE': mae, 'MSE': mse, 'RMSE': rmse, 'MAPE': mape, 'R2': r2, 'EVS': evs}

def plot_predictions(y_true, y_pred, model_name='LSTM', n_points=96):
    """
    Parameters
    ----------
    y_true     : ndarray, actual traffic values (real scale)
    y_pred     : ndarray, predicted traffic values (real scale)
    model_name : str
    n_points   : int  number of time steps to show (96 = 1 full day)
    """
    
    # Create time axis: one full day at 15-min intervals
    date = '2006-10-01 00:00'
    x = pd.date_range(date, periods=n_points, freq='15min')
    
    fig, ax = plt.subplots(figsize=(14,5))
    
    ax.plot(x, y_true[:n_points], label='Actual')
    ax.plot(x, y_pred[:n_points], label='Predicted', 
            linewidth=1.5, linestyle='--', color='steelblue')
    
    plt.title(f'{model_name} - Predicted vs Actual Traffic Flow (1 Day)')
    plt.xlabel('Time of Day')
    plt.ylabel('Traffic Flow (vehicles / 15 mins)')
    plt.legend()
    plt.grid(True)
    
    # Format x-axis as HH:MM
    date_format = mpl.dates.DateFormatter("%H:%M")
    ax.xaxis.set_major_formatter(date_format)
    fig.autofmt_xdate()
    
    plt.tight_layout()
    
    os.makedirs('images', exist_ok=True)
    save_path = f'images/{model_name.lower()}_predicting.png'
    plt.savefig(save_path)
    print(f"\nPrediction plot saved to: {save_path}")
    plt.show()

def plot_loss_from_csv(model_name='lstm'):
    """
    Load the saved loss CSV and re-plot the training loss curve.
    Useful if you want to re-generate the plot without retraining.
    """
    
    csv_path = f'model/{model_name}_loss.csv'
    if not os.path.exists(csv_path):
        print(f"Loss CSV not found at {csv_path} - run train_lstm.py to try again.")
        return 
    
    df = pd.read_csv(csv_path)
    
    plt.figure(figsize=(10,4))
    plt.plot(df['loss'], label='Train Loss')
    plt.plot(df['val_loss'], label='Val Loss')
    plt.title(f'{model_name.upper()} Training Loss Over Epochs')
    plt.xlabel('Epoch')
    plt.ylabel('Loss (MSE)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    
    save_path = f'images/{model_name}_loss.png'
    plt.savefig(save_path)
    print(f"Loss plot saved to: {save_path}")
    plt.show()

def get_all_scats_numbers(file_path):
    df = read_data(file_path)
    return sorted(df['SCATS Number'].unique())

def main():
    scats_numbers = get_all_scats_numbers(DATA_FILE)
    all_metrics = []
    
    for scats_id in scats_numbers:
        model_name = f'lstm_{scats_id}'
        model_path = f'model/{model_name}.h5'
        
        if not os.path.exists(model_path):
            print(f"Model not found for site {scats_id} - skipping")
            continue
    
        print(f"\n--- Evaluating LSTM for SCATS site {scats_id} ---")
        # 1. Load and process data (same settings as training)
        X_train, y_train, X_test, y_test, scaler = process_data(
            file_path = DATA_FILE,
            lag = 12,
            scats_number=scats_id
        )
        
        # 2. Reshape X_test to 3D for LSTM: (samples, time_steps, features)
        X_test_3d = np.reshape(X_test, (X_test.shape[0], X_test.shape[1], 1))
        
        # 3. Inverse transform y_test back to real traffic values
        y_true = scaler.inverse_transform(y_test.reshape(-1, 1)).flatten()
        
        model = load_model(model_path, compile=False)
        model.compile(loss='mse', optimizer='adam')
        print(f"Model loaded from {model_path}")
        
        # 5. Predict
        y_pred_scaled = model.predict(X_test_3d)
        
        # 6. Inverse transform predictions back to real traffic values
        y_pred = scaler.inverse_transform(y_pred_scaled.reshape(-1, 1)).flatten()
        
        # 7.Evaluate
        metrics = evaluate(y_true, y_pred, model_name=model_name)
        metrics['scats_id'] = scats_id
        all_metrics.append(metrics)
        
        # 8. Plot predicted vs actual
        plot_predictions(y_true, y_pred, model_name=model_name)
        
        # 9. Re-plot loss curve from saved CSV
        plot_loss_from_csv(model_name=model_name)
    
    if all_metrics:
        print("\n--- Summary: All SCATS Sites ---")
        df_summary = pd.DataFrame(all_metrics)
        df_summary = df_summary.set_index('scats_id')
        print(df_summary.to_string())
        df_summary.to_csv('model/lstm_all_sites_metrics.csv')
        print("\nSummary saved to model/lstm_all_sites_metrics.csv")

if __name__ == '__main__':
    main()
