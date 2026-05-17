"""
Train the NN model.
"""
import sys
import os
import warnings
import argparse
import numpy as np
import pandas as pd
from data import process_data
from model import model
from keras.models import Model
from keras.callbacks import EarlyStopping
from keras.callbacks import ModelCheckpoint
warnings.filterwarnings("ignore")


def train_model(model, X_train, y_train, name, config, scat, location):
    """train
    train a single model.

    # Arguments
        model: Model, NN model to train.
        X_train: ndarray(number, lags), Input data for train.
        y_train: ndarray(number, ), result data for train.
        name: String, name of model.
        config: Dict, parameter for train.
    """

    model.compile(loss="mse", optimizer="rmsprop", metrics=['mae'])
    checkpoint_path = f"model/{scat}-{location}-{name}.h5"
    #used to stop model early if no changes to value loss after 30 epochs
    early = EarlyStopping(monitor='val_loss', patience=30, verbose=0, mode='auto', restore_best_weights=False)
    #saving best model incase model starts to overfit.
    checkpoint = ModelCheckpoint(checkpoint_path, monitor="val_loss", mode="min", save_best_only=True, verbose=1)
    hist = model.fit(
        X_train, y_train,
        batch_size=config["batch"],
        epochs=config["epochs"],
        validation_split=0.05,
        callbacks =[early, checkpoint])
    #saving model under model directiory with the scat number and loc ation the model is trained on
    df = pd.DataFrame.from_dict(hist.history)
    df.to_csv('model/' + name + ' loss.csv', encoding='utf-8', index=False)




def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        default="gru",
        help="Model to train.")
    args = parser.parse_args()

    lag = 12
    config = {"batch": 256, "epochs": 600}
    #loop for creating a model for every location in every scat
    for scat in sorted(os.listdir("../SCATS_Data"), key=int):
        for location in os.listdir(os.path.join("../SCATS_Data", scat)):
            file1 = os.path.join("../SCATS_Data", scat, location, "train.csv")
            file2 = os.path.join("../SCATS_Data", scat, location, "test.csv")
            X_train, y_train, _, _, _, _, _ = process_data(file1, file2, lag)


            if args.model == 'gru':
                X_train = np.reshape(X_train, (X_train.shape[0], X_train.shape[1], 1))
                m = model.get_gru([12, 64, 64, 1])
                train_model(m, X_train, y_train, args.model, config, scat, location)



if __name__ == '__main__':
    main(sys.argv)
