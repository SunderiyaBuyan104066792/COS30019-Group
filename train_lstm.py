import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from keras.callbacks import EarlyStopping
from data.process_data import process_data_multi, read_data
from lstm_model import get_lstm
from cluster_config import get_cluster_map

# Settings
DATA_FILE = "data/Scats_Data_Oct_2006.xls"
N_CLUSTERS = 5
UNITS = [12, 64, 64, 1]
CONFIG = {
    'batch_size': 32,
    'epochs': 200,
}


def train_model(model, X_train, y_train, name, config):
    """
    - Parameters:
        model   : Keras model returned by get_lstm()
        X_train : ndarray, shape (N, lag, 1)   — 3D input for LSTM
        y_train : ndarray, shape (N,)
        name    : str   used for filenames e.g. 'lstm'
        config  : dict  with keys 'batch_size' and 'epochs'
    """
    
    # Compile — tell the model how to measure error and how to improve
    # loss='mse'       : Mean Squared Error, standard for regression
    # optimizer='adam' : adaptive learning rate, works well in most cases
    # metrics=['mae']  : also track Mean Absolute Error during training
    model.compile(loss='mse', optimizer='adam', metrics=['mae'])
    
    print(f"\nTraining {name.upper()} ...")
    print(f"  Epochs:     {config['epochs']}")
    print(f"  Batch size: {config['batch_size']}")
    print(f"  Train size:  {len(X_train)}\n")
    
    early_stop = EarlyStopping(
        monitor='val_loss',
        patience=10,
        restore_best_weights=True,
        verbose=1
    )
    
    hist = model.fit(
            X_train, y_train,
            batch_size=config['batch_size'],
            epochs=config['epochs'],
            validation_split=0.05,
            callbacks=[early_stop],
            verbose=1
    )
    
    os.makedirs('model', exist_ok=True)
    save_path = os.path.join('model', name + '.h5')
    model.save(save_path)
    print(f"\nModel saved to: {save_path}")
    
    df_loss = pd.DataFrame(hist.history)
    csv_path = os.path.join('model', name + '_loss.csv')
    df_loss.to_csv(csv_path, index=False)
    print(f"Loss history saved to: {csv_path}")
    
    return hist

def plot_loss(hist, name):
    # Plot training loss vs validation loss and save as image.
    
    plt.figure(figsize=(10, 4))
    plt.plot(hist.history['loss'], label='Train Loss')
    plt.plot(hist.history['val_loss'], label='Val Loss')
    plt.title(f'{name.upper()} Training Loss Over Epochs')
    plt.xlabel('Epoch')
    plt.ylabel('Loss (MSE)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    
    os.makedirs('images', exist_ok=True)
    save_path = f'images/{name}_loss.png'
    plt.savefig(save_path)
    print(f"Loss plot saved to: {save_path}")
    plt.show()
    plt.pause(5)
    plt.close()

def main():
    # 1. Build cluster map
    cluster_map = get_cluster_map(DATA_FILE, n_clusters=N_CLUSTERS)
    print(f"\nFound {len(cluster_map)} geographic clusters:")
    for cid, scats_list in cluster_map.items():
        print(f"   Cluster {cid}: {len(scats_list)} sites: {scats_list}")

    # Print model structure once before the loop
    print()
    get_lstm(UNITS).summary()
    
    total_clusters = len(cluster_map)
    
    # 2. Train one model per cluster
    for cluster_id, scats_list in cluster_map.items():
        model_name = f'lstm_cluster_{cluster_id}'
        print(f"\n{'-'*10}")
        print(f"   Cluster {cluster_id + 1}/{total_clusters} - {model_name}")
        print(f"   Sites: {scats_list}")
        print(f"{'-'*10}")
        
        # Pool data from all sites in this cluster
        X_train, y_train, X_test, y_test, scaler = process_data_multi(
            file_path=DATA_FILE,
            scats_list=scats_list,
            lag=4
        )
        
        # LSTM needs 3-D input: (samples, time_steps, features)
        X_train_3d = X_train.reshape(X_train.shape[0], X_train.shape[1], 1)
        
        # Fresh model for each cluster (no weight leakage between clusters)
        model = get_lstm(UNITS)
        
        hist = train_model(model, X_train_3d, y_train, model_name, CONFIG)
        plot_loss(hist, model_name)
        
        print(f"\n   Cluster {cluster_id} done.")
    
    print(f"{'-'*10}")
    print(f"\nAll {total_clusters} cluster models trained.")
    print(f"Run main_lstm.py to evaluate them.")
    print(f"{'-'*10}")

if __name__ == '__main__':
    main()
