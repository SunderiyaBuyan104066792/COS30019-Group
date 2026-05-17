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
from data.process_data import process_data_multi, read_data
from cluster_config import get_cluster_map

# Settings
DATA_FILE = 'data/Scats_Data_Oct_2006.xls'
N_CLUSTERS = 5   # Can be changed but must match cluster_config.N_CLUSTERS


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
            linewidth=1.5, color='red')
    
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
    plt.pause(5)
    plt.close()

def plot_loss_from_csv(model_name='lstm'):
    # Load the saved loss CSV and re-plot the training loss curve.
    
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
    plt.pause(5)
    plt.close()

def main():
    cluster_map = get_cluster_map(DATA_FILE, n_clusters=N_CLUSTERS)
    all_metrics = []
    all_predictions = []
    
    for cluster_id, scats_list in cluster_map.items():
        model_name = f'lstm_cluster_{cluster_id}'
        model_path = f'model/{model_name}.h5'
        
        print(f"{'-'*10}")
        print(f"   Cluster: {cluster_id} - {model_name}")
        print(f"   Sites: {scats_list}")
        print(f"{'-'*10}")
        
        if not os.path.exists(model_path):
            print(f"\n  Model not found at {model_path} - skipping")
            continue
        
        # 1. Reconstruct test data (same settings as training)
        _, _, X_test, y_test, scaler = process_data_multi(
            file_path=DATA_FILE,
            scats_list=scats_list,
            lag=4
        )
        
        # 2. Load model (compile=False avoids legacy .h5 deserialization bug)
        model= load_model(model_path, compile=False)
        model.compile(loss='mse', optimizer='adam')
        print(f"\n  Model loaded from {model_path}")
        
        # 3. Reshape for LSTM and predict
        X_test_3d = X_test.reshape(X_test.shape[0], X_test.shape[1], 1)
        
        y_true_scaled = y_test
        y_pred_scaled = model.predict(X_test_3d)
        
        # 4. Inverse-transform to real vehicle count
        y_true = scaler.inverse_transform(y_true_scaled.reshape(-1, 1)).flatten()
        y_pred = scaler.inverse_transform(y_pred_scaled.reshape(-1, 1)).flatten()
        
        # Store for summary plot
        all_predictions.append({        
            'cluster_id': cluster_id,
            'y_true': y_true,
            'y_pred': y_pred,
        })
        
        # 5. Evaluate
        metrics = evaluate(y_true, y_pred, model_name=model_name)
        metrics['cluster_id'] = cluster_id
        metrics['n_sites'] = len(scats_list)
        metrics['scats_sites'] = str(scats_list)
        all_metrics.append(metrics)
        
        # 6. Plots
        plot_predictions(y_true, y_pred, model_name=model_name)
        plot_loss_from_csv(model_name)
    
    # Summary
    if all_metrics:
        print(f"\n{'-'*10}")
        print("Summary: All Clusters Models")
        print(f"{'-'*10}")
        df_summary = pd.DataFrame(all_metrics).set_index('cluster_id')
        print(df_summary[['n_sites', 'MAE', 'RMSE', 'MAPE', 'R2', 'EVS']].to_string())
        
        os.makedirs('model', exist_ok=True)
        out_path = 'model/lstm_cluster_metrics.csv'
        df_summary.to_csv(out_path)
        print(f"\nSummary saved to {out_path}")

if __name__ == '__main__':
    main()
