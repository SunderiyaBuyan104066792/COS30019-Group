import os
import re
import pandas as pd

def parse_roads(location):
    parts = re.split(r'\s+[NSEW]{1,2}\s+of\s+', location, flags=re.IGNORECASE)
    return '|'.join(p.strip() for p in parts if p.strip())


FIXES = {
    4266: (-37.8253, 145.0435)
}

def apply_coordinate_fixes(metadata):
    for scats_num, (lat, lon) in FIXES.items():
        mask = metadata['scats_num'] == scats_num
        metadata.loc[mask, 'latitude'] = lat
        metadata.loc[mask, 'longitude'] = lon
    return metadata


def build_metadata():
    df = pd.read_excel('../data/Scats Data October 2006.xls', sheet_name='Data', engine='calamine', header=1)

    coords = df.groupby('SCATS Number')[['NB_LATITUDE', 'NB_LONGITUDE']].mean().reset_index()
    coords.columns = ['scats_num', 'latitude', 'longitude']

    roads = (df.groupby('SCATS Number')['Location']
               .apply(lambda locs: '|'.join(
                   set(r for loc in locs for r in parse_roads(loc).split('|'))
               ))
               .reset_index())
    roads.columns = ['scats_num', 'roads']

    metadata = coords.merge(roads, on='scats_num')
    metadata = apply_coordinate_fixes(metadata)
    metadata.to_csv('site_road_data/road_data.csv', index=False)
    print(f'Saved road_data.csv ({len(metadata)} sites)')
    print(metadata.head(3).to_string())


if __name__ == '__main__':
    build_metadata()