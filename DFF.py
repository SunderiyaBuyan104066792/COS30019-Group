import sys
import warnings
import argparse
import numpy as np
import pandas as pd
from keras.models import Model
from keras.callbacks import EarlyStopping
import tensorflow as tf 
import keras.layers as layers
from keras.layers import Dense, Dropout, Flatten, Concatenate


# this model will be a combination of a Deep Feed Forward model and Deep Conv Network (DCN)

# because it has correlated and uncorrelated features (the time data is correlated, while the location and date do not effect themselves or eachother in the same capacity)
# DCN heavily relies on the use of features to make predictions- more predicting based on charactistics
# DFF is just a more in-depth perceptron- very straight forward- predicting using the prior time interval


def CustomModel(n_lags=12):
    # location, day, time
    input_features = layers.Input(shape=(3,))            


    # last n_lags traffic counts
    input_lags = layers.Input(shape=(n_lags, 1))         


    x = layers.Conv1D(2, kernel_size=8, padding='same', activation='relu')(input_lags)
    x = Dropout(0.15)(x)

    x = Flatten()(x)

    x = Concatenate()([input_features, x])

    x = Dense(4, activation='relu')(x)

    output = Dense(1, activation = 'relu')(x)  # linear output for regression

    return Model(inputs=[input_features, input_lags], outputs=output)





    
















