import sys
import numpy as np
import pandas as pd
from sklearn.cluster import KMeans


# Settings
N_CLUSTERS = 5
RANDOM_STATE = 42


def get_site_coords(file_path):
    df = pd.read_excel(file_path, sheet_name='Data', engine='xlrd', header=1)
    df = df.iloc[1:].reset_index(drop=True)
    
    df['NB_LATITUDE'] = df['NB_LATITUDE'].replace(0, float('nan'))
    df['NB_LONGITUDE'] = df['NB_LONGITUDE'].replace(0, float('nan'))
    
    sites = (
        df.groupby('SCATS Number').agg(Location=('Location', 'first'),
                                        Lat=('NB_LATITUDE', 'mean'),
                                        Lon=('NB_LONGITUDE', 'mean')).reset_index()
    )
    return sites

def build_clusters(file_path, n_clusters = N_CLUSTERS, random_state = RANDOM_STATE):
    sites = get_site_coords(file_path)
    
    coords = sites[['Lat', 'Lon']].values
    km = KMeans(n_clusters=n_clusters, random_state=random_state, n_init=10)
    sites = sites.copy()
    sites['cluster'] = km.fit_predict(coords)
    
    sites = sites.sort_values(['cluster', 'SCATS Number']).reset_index(drop=True)
    return sites

def get_cluster_map(file_path, **kwargs):
    df = build_clusters(file_path, **kwargs)
    cluster_map = {}
    for cluster_id, group in df.groupby('cluster'):
        cluster_map[int(cluster_id)] = sorted(group['SCATS Number'].to_list())
    return cluster_map

if __name__ == '__main__':
    path = sys.argv[1] if len(sys.argv) > 1 else 'data/Scats_Data_Oct_2006.xls'
    cluster_map = get_cluster_map(path)
    total = sum(len(v) for v in cluster_map.values())
    print(f"\nGeographic clusters ({N_CLUSTERS} areas, {total} sites total:\n)")
    for cid, scats_list in cluster_map.items():
        print(f"   Cluster {cid}  ({len(scats_list)} sites): {scats_list}")