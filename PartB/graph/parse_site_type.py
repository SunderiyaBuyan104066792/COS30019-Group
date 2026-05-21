import os
import pandas as pd

def get_sites():
    return [int(s) for s in os.listdir('../data/SCATS_Data')
            if os.path.isdir(os.path.join('../data/SCATS_Data', s))]


def parse_site_types():
    df = pd.read_excel('site_road_data/SCATSSiteListingSpreadsheet_VicRoads.xls', sheet_name='SCATS Site Numbers', engine='calamine', header=8)
    df.columns = ['site_num', 'location', 'site_type', 'directory', 'map_ref']

    df = df.dropna(subset=['site_num'])
    df['site_num'] = pd.to_numeric(df['site_num'], errors='coerce')
    df = df.dropna(subset=['site_num'])
    df['site_num']  = df['site_num'].astype(int)
    df['site_type'] = df['site_type'].str.strip()

    return df[['site_num', 'site_type']]


def main():
    boroondara = get_sites()
    all_sites = parse_site_types()

    result = all_sites[all_sites['site_num'].isin(boroondara)].copy()
    result['is_intersection'] = (result['site_type'] == 'INT').astype(int)
    result = result.sort_values('site_num').reset_index(drop=True)

    result.to_csv('site_road_data/site_types.csv', index=False)
    print('\nSaved site_types.csv')

if __name__ == '__main__':
    main()