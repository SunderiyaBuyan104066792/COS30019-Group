import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import DFF



# not using h5 - using keras instead


# lags 12 => takes the last 3 hours at 15 min intervals
N_LAGS = 12 



# take in training x=> labels, y=> data
x = np.loadtxt('train_x.csv').astype(np.float32)
y = np.loadtxt('train_y.csv').astype(np.float32)

# Build the lagged-target matrix. Data is sorted by (location, day, time),
# continuous within each location, so a row's 12 prior rows are its lags —
# as long as they're from the same location.
locations = x[:, 0]
lags = np.zeros((len(y), N_LAGS), dtype=np.float32)
for k in range(N_LAGS):
    # column 0 = oldest lag, column 11 = most recent
    offset = N_LAGS - k        
    lags[offset:, k] = y[:len(y) - offset]

# A row is valid if the row 12 steps back is the same location (contiguity
# within a location means everything in between is also the same location).
valid = np.zeros(len(y), dtype=bool)
valid[N_LAGS:] = locations[:-N_LAGS] == locations[N_LAGS:]
x, lags, y = x[valid], lags[valid], y[valid]

# Normalize. Lags share the target's scale so use y's stats for them too.
x_mean, x_std = x.mean(0), x.std(0) + 1e-8
y_mean, y_std = y.mean(), y.std() + 1e-8
x_n = (x - x_mean) / x_std
y_n = (y - y_mean) / y_std
lags_n = ((lags - y_mean) / y_std)[..., None]  # add channel dim for Conv1D

# Shuffle so validation_split doesn't carve off entire locations.
rng = np.random.default_rng(42)
perm = rng.permutation(len(y_n))
x_n, lags_n, y_n = x_n[perm], lags_n[perm], y_n[perm]

model = DFF.CustomModel(n_lags=N_LAGS)
model.compile(optimizer=tf.keras.optimizers.Adam(1e-4),
    loss='mse', metrics=['mse'])

history = model.fit([x_n, lags_n], y_n,
    epochs=50, batch_size=512, validation_split=0.2)

plt.plot(history.history['loss'], label='train loss')
plt.plot(history.history['val_loss'], label='val loss')
plt.xlabel('epoch')
plt.ylabel('loss (normalized MSE)')
plt.legend()
#plt.savefig('loss.png', dpi=150)
plt.show()

model.save('model.keras')

# Save normalization stats so inference can undo them later
np.savez('norm.npz', x_mean=x_mean, x_std=x_std, y_mean=y_mean, y_std=y_std)
