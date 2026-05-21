import os
import re
import pandas as pd

# NOTE: Models are trained per direction —> a 4-way intersection produces 4 models
#       per ML algorithm.
#
# Steps:
#   1. Load dataset
#   2. Format V00-V95 columns into individual rows
#   3. Split by SCATS site and direction (location), 80% train / 20% test
#   4. Save per-direction train.csv and test.csv

def process_df():
    df = pd.read_excel('Scats Data October 2006.xls', sheet_name='Data', engine='calamine', header=1)
    df['Date'] = pd.to_datetime(df['Date'])

    v_cols = [f'V{i:02d}' for i in range(96)]
    df = df.melt(
        id_vars=['SCATS Number', 'Location', 'Date'],
        value_vars=v_cols,
        var_name='interval',
        value_name='volume'
    )
    df['datetime'] = df['Date'] + pd.to_timedelta(df['interval'].str[1:].astype(int) * 15, unit='min')
    df = df.drop(columns=['Date', 'interval']).sort_values(['SCATS Number', 'Location', 'datetime'])

    cols = ['datetime', 'volume']
    for (site, location), group in df.groupby(['SCATS Number', 'Location']):
        group = group.sort_values('datetime').reset_index(drop=True)
        split = int(len(group) * 0.8)

        road_dir = os.path.join('SCATS_Data', str(site), re.sub(r'\s+', '_', location.strip()))
        os.makedirs(road_dir, exist_ok=True)

        group.iloc[:split][cols].to_csv(os.path.join(road_dir, 'train.csv'), index=False)
        group.iloc[split:][cols].to_csv(os.path.join(road_dir, 'test.csv'),  index=False)
        print(f'{site}|{location}')


if __name__ == '__main__':
    process_df()