from keras.layers import Dense, Dropout, Activation, LSTM
from keras.models import Sequential

def get_lstm(units):
    """
    Build and return a stacked LSTM model.

    Parameter
    units : list [input_steps, hidden1, hidden2, output]
            e.g. [12, 64, 64, 1]

    Return
    model : Keras Sequential model
    """
    
    model = Sequential()
    
    # First layer - return_sequences=True because next layer is also LSTM 
    model.add(LSTM(units[1], input_shape=(units[0], 1), return_sequences=True))
    # Second layer
    model.add(LSTM(units[2]))
    # Dropout before output layer to prevent overfitting
    model.add(Dropout(0.2))
    # Output layer
    model.add(Dense(units[3], activation='sigmoid'))
    
    return model
