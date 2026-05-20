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
    # ssequence of n_lags past vehicle counts
    input_lags = layers.Input(shape=(n_lags, 1))         

    # 2 filters, kernel size 8 (size of scope), padding is same (keeps as 12)
    x = layers.Conv1D(2, kernel_size=8, padding='same', activation='relu')(input_lags)

    # 15% dropout
    x = Dropout(0.15)(x)

    # converting multiple dimensional array in 1D: 
    # https://www.geeksforgeeks.org/machine-learning/impact-of-image-flattening/
    x = Flatten()(x)

    # joins the base input_features with our theta
    x = Concatenate()([input_features, x])

    # dense layers our the DFF part of the model
    # hidden relu layer -> input of 27 into bottleneck of 4
    x = Dense(4, activation='relu')(x)

    # another dense layer -> this is our output layer (bottleneck of 1)
    output = Dense(1, activation = 'relu')(x) 

    return Model(inputs=[input_features, input_lags], outputs=output)





    
















