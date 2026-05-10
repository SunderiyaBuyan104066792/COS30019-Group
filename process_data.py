import numpy as np
import pandas as pd


def load_data(filepath):
    """
    Load the raw XLS file and return a cleaned DataFrame.
    """
    df = pd.read_excel(filepath, sheet_name='Data', engine='xlrd', header=1)
    df = df.iloc[1:]  # skip metadata row
    df = df.reset_index(drop=True)
    return df


def extract_inputs(df):
    """
    Build the input matrix [location_id, day_of_week, time_slot]
    and target array y (traffic flow values).

    Returns
    -------
    inputs : ndarray, shape (num_rows * 96, 3)
    y      : ndarray, shape (num_rows * 96,)
    """
    locations = df['Location'].values
    dates     = df['Date'].values

    # Map each unique location to a numeric ID
    unique_locations = np.unique(locations)
    x1 = np.zeros(len(locations))
    for i, loc in enumerate(unique_locations):
        x1[np.where(loc == locations)[0]] = i

    # Convert date to day-of-week (0=Mon, 6=Sun)
    unique_dates = np.unique(dates)
    x2 = np.zeros(len(dates))
    for i, d in enumerate(unique_dates):
        x2[np.where(d == dates)[0]] = pd.Timestamp(d).dayofweek

    # Expand each row into 96 time slots
    num_rows    = len(locations)
    x1_expanded = np.zeros(num_rows * 96)
    x2_expanded = np.zeros(num_rows * 96)
    x3          = np.zeros(num_rows * 96)

    for row_idx in range(num_rows):
        for slot in range(96):
            out_idx                = row_idx * 96 + slot
            x1_expanded[out_idx]  = x1[row_idx]
            x2_expanded[out_idx]  = x2[row_idx]
            x3[out_idx]           = slot

    # Extract traffic flow values V00-V95 as target y
    flow_cols = ['V%02d' % i for i in range(96)]
    y = df[flow_cols].values.flatten().astype(float)

    # Combine into input matrix
    inputs = np.array([x1_expanded, x2_expanded, x3]).T

    return inputs, y


def process_data(filepath):
    """
    Full pipeline — call this from train.py or main.py.

    Returns
    -------
    inputs : ndarray, shape (N, 3)   [location_id, day_of_week, time_slot]
    y      : ndarray, shape (N,)     traffic flow values
    """
    df           = load_data(filepath)
    inputs, y    = extract_inputs(df)

    print(f"Inputs shape: {inputs.shape}")
    print(f"Target shape: {y.shape}")

    return inputs, y


# ── Quick test ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    inputs, y = process_data('Scats Data October 2006.xls')
    print("\nFirst 10 rows [location_id, day_of_week, time_slot]:")
    print(inputs[:10])
    print("\nFirst 10 traffic flow values:")
    print(y[:10])