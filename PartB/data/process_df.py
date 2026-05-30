import os
import re
import pandas as pd

def process_df():
    xls_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Scats Data October 2006.xls')
    df = pd.read_excel(xls_path, sheet_name='Data', engine='calamine', header=1)
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

        road_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'SCATS_Data', str(site), re.sub(r'\s+', '_', location.strip()))
        os.makedirs(road_dir, exist_ok=True)

        group.iloc[:split][cols].to_csv(os.path.join(road_dir, 'train.csv'), index=False)
        group.iloc[split:][cols].to_csv(os.path.join(road_dir, 'test.csv'), index=False)
        print(f'{site}|{location}')


if __name__ == '__main__':
    process_df()
