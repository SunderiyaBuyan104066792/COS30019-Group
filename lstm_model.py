from keras.layers import Dense, Dropout, Activation, LSTM
from keras.models import Sequential

def get_lstm(units):
    """
    Parameter
        units: List (int)
    Return
        model: Model
    """
    
    model = Sequential()
    
    # First layer
    model.add(LSTM(units[1], input_shape=(units[0], 1), return_sequences=True))
    # Second layer
    model.add(LSTM(units[2]))
    model.add(Dense(units[3], activation='sigmoid'))
    model.add(Dropout(0.2))
    
    return model