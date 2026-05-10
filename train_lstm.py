import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from data.process_data import process_data
from lstm_model import get_lstm

# Settings
DATA_FILE = "Scats_Data_Oct_2006.xls"
UNITS = [12, 64, 64, 1]
CONFIG = {
    'batch_size': 32,
    'epochs': 50,
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
    # loss='mse'       → Mean Squared Error, standard for regression
    # optimizer='adam' → adaptive learning rate, works well in most cases
    # metrics=['mae']  → also track Mean Absolute Error during training
    model.compile(loss='mse', optimizer='adam', metrics=['mape'])
    
    print(f"\nTraining {name.upper()} model...")
    print(f"  Epochs:     {config['epochs']}")
    print(f"  Batch size: {config['batch_size']}")
    print(f"  Train size:  {len(X_train)}\n")
    
    hist = model.fit(X_train, y_train,
            batch_size=config['batch_size'],
            epochs=config['epochs'],
            validation_split=0.05,
            verbose=1
    )
    
    os.makedirs('model', exist_ok=True)
    save_path = os.path.join('model', name + '.h5')
    model.save(save_path)
    print(f"\nModel saved to: {save_path}")
    
    df_loss = pd.DataFrame(hist.history)
    csv_path = os.path.join('model', name + '_loss.csv')
    df_loss.to_csv(csv_path, index=False, encoding='utf-8')
    print(f"Loss history saved to: {csv_path}")
    
    return hist

def plot_loss(hist, name):
    """
    Plot training loss vs validation loss and save as image.
    Use this graph in your report's Insights section.
    """
    
    plt.figure(figsize=(10, 4))
    plt.plot(hist.history['loss'], label='Train Loss')
    plt.plot(hist.history['val_loss'], label='Val Loss')
    plt.title(f'{name.upper()} Training Loss Over Epochs')
    plt.xlabel('Epoch')
    plt.ylabel('Loss (MSE)')
    plt.legend
    plt.grid(True)
    plt.tight_layout()
    
    os.makedirs('images', exist_ok=True)
    save_path = f'images/{name}_loss.png'
    plt.savefig(save_path)
    print(f"Loss plot saved to: {save_path}")
    plt.show()

def main():
    # Load and process data
    X_train, y_train, X_test, y_test, scaler = process_data(
        file_path=DATA_FILE,
        lag = 12
    )
    
    # 2. Reshape X to 3D — LSTM requires shape (samples, time_steps, features)
    #    Currently X is (N, 12), we need (N, 12, 1)
    X_train = np.reshape(X_train, (X_train.shape[0], X_train.shape[1], 1))
    X_test = np.reshape(X_test, (X_test.shape[0], X_test.shape[1], 1))
    
    # 3. Build the LSTM model
    model = get_lstm(UNITS)
    model = get_lstm(UNITS)
    model.summary()
    
    # 4. Train and save
    hist = train_model(model, X_train, y_train, 'lstm', CONFIG)
    
    # 5. Plot loss curve
    plot_loss(hist, 'lstm')
    
    print(f"\nFinished. Run lstm_main.py to evaluate the model.")

if __name__ == '__main__':
    main()