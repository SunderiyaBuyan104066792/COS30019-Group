from keras.layers import Dense, Dropout, Flatten, Concatenate, Input, Conv1D
from keras.layers import LSTM, GRU
from keras.models import Sequential, Model



def get_lstm(units):
    """LSTM(Long Short-Term Memory)
    Build LSTM Model.

    # Arguments
        units: List(int), [lags, hidden1, hidden2, output]
    # Returns
        model: Model, nn model.
    """
    model = Sequential()
    model.add(LSTM(units[1], input_shape=(units[0], 1), return_sequences=True))
    model.add(LSTM(units[2]))
    model.add(Dropout(0.2))
    model.add(Dense(units[3], activation='sigmoid'))

    return model


def get_gru(units):
    """GRU(Gated Recurrent Unit)
    Build GRU Model.

    # Arguments
        units: List(int), [lags, hidden1, hidden2, output]
    # Returns
        model: Model, nn model.
    """
    model = Sequential()
    model.add(GRU(units[1], input_shape=(units[0], 1), return_sequences=True))
    model.add(GRU(units[2]))
    model.add(Dropout(0.2))
    model.add(Dense(units[3], activation='sigmoid'))

    return model


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

    # 2 filters, kernel size 3 (size of scope), padding is same
    x = Conv1D(2, kernel_size=3, padding='same', activation='relu')(input_lags)

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
