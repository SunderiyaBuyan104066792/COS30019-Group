from keras.layers import Dense, Dropout, Flatten, Concatenate, Input, Conv1D
from keras.layers import LSTM, GRU
from keras.models import Model


def get_lstm(n_lags):
    """LSTM with time features
    Same two-layer LSTM that accepts day-of-week and
    time-of-day as a second input that is concatenated after the recurrent
    layers, before the output Dense.

    # Arguments
        n_lags: int, number of past time steps fed in.
    # Returns
        model: Model, nn model.
    """
    input_features = Input(shape=(2,))
    input_lags = Input(shape=(n_lags, 1))

    x = LSTM(64, return_sequences=True)(input_lags)
    x = LSTM(64)(x)
    x = Dropout(0.2)(x)

    x = Concatenate()([input_features, x])
    output = Dense(1, activation='sigmoid')(x)

    return Model(inputs=[input_features, input_lags], outputs=output)


def get_gru(n_lags):
    """GRU with time features
     Same two-layer LSTM that accepts day-of-week and
    time-of-day as a second input that is concatenated after the recurrent
    layers, before the output Dense.

    # Arguments
        n_lags: int, number of past time steps fed in.
    # Returns
        model: Model, nn model.
    """
    input_features = Input(shape=(2,))
    input_lags = Input(shape=(n_lags, 1))

    x = GRU(64, return_sequences=True)(input_lags)
    x = GRU(64)(x)
    x = Dropout(0.2)(x)

    x = Concatenate()([input_features, x])
    output = Dense(1, activation='sigmoid')(x)

    return Model(inputs=[input_features, input_lags], outputs=output)


def get_custom(n_lags):
    """Custom
    -> it is a combination of Deep Feed Forward (in-depth perceptron) and convolutional neural network

    # because it has correlated and uncorrelated features (the time data is correlated, while the location and date do not effect themselves or eachother in the same capacity)
    # DCN heavily relies on the use of features to make predictions- more predicting based on charactistics
    # DFF is just a more in-depth perceptron- very straight forward- predicting using the prior time interval

    """
    # day, time -> because we are training per location, it is no longer an input
    input_features = Input(shape=(2, ))

    # last n_lags traffic counts
    # sequence of n_lags past vehicle counts
    input_lags = Input(shape=(n_lags, 1))

    # 2 filters(changed to 64), kernel size 3 (size of scope), padding is same
    x = Conv1D(64, kernel_size=3, padding='same', activation='relu')(input_lags)

    # 15% dropout
    x = Dropout(0.15)(x)

    # converting multiple dimensional array in 1D:
    # https://www.geeksforgeeks.org/machine-learning/impact-of-image-flattening/
    x = Flatten()(x)

    # joins the base input_features with our theta
    x = Concatenate()([input_features, x])

    # dense layers our the DFF part of the model
    # hidden relu layer -> input of 27 into bottleneck of 4
    x = Dense(64, activation='relu')(x)
    x = Dropout(0.2)(x)
    x = Dense(32, activation='relu')(x)

    # another dense layer -> this is our output layer (bottleneck of 1)
    output = Dense(1, activation = 'relu')(x)

    return Model(inputs=[input_features, input_lags], outputs=output)
