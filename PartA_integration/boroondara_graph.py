import math
import xlrd

MAX_EDGE_KM = 3.0

def haversine_km(lat1, lon1, lat2, lon2):
    # Great-circle distance in kilometers between two GPS points
    
    R = 6371.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def load_scats_data(file_path):
    """
    Reads the VicRoads Excel file and extracts one entry per unique SCATS site.

    - Returns:
    dict  {scats_id (int): (lat (float), lon (float), location (str))}
    """
    
    wb = xlrd.open_workbook(file_path)
    sh = wb.sheet_by_name('Data')
    sites = {}
    for i in range(2, sh.nrows):
        row = sh.row_values(i)
        if not row[0]:
            continue
        sid = int(float(str(row[0]).strip()))
        if sid not in sites:
            sites[sid] = (float(row[3]), float(row[4]), str(row[1]).strip())
    
    return sites

def build_graph(scats_sites, max_edge_km=MAX_EDGE_KM):
    """
    Builds nodes and edges dicts compatible with search.py.

    - Parameters:
    scats_sites : dict  {scats_id: (lat, lon, location)}
    max_edge_km : float  maximum distance to connect two nodes

    - Returns
    nodes : {scats_id: (x, y)}  coordinates scaled x100
    edges : {from_id: [(to_id, dist_km), ...]}  bidirectional, sorted by dest id
    """
    
    ids = sorted(scats_sites.keys())
    nodes = {sid: (round(scats_sites[sid][1]*100, 4), 
                    round(scats_sites[sid][0]*100, 4)) for sid in ids}
    edges = {sid: [] for sid in ids}
    for i in range(len(ids)):
        for j in range(i+1, len(ids)):
            a, b = ids[i], ids[j]
            dist = haversine_km(scats_sites[a][0], scats_sites[a][1],
                                scats_sites[b][0], scats_sites[b][1])
            if dist <= max_edge_km:
                edges[a].append((b, dist))
                edges[b].append((a, dist))
    for sid in ids:
        edges[sid].sort(key=lambda x: x[0])
    
    return nodes, edges

if __name__ == '__main__':
    DATA = 'data/Scats_Data_Oct_2006.xls'
    sites = load_scats_data(DATA)
    nodes, edges = build_graph(sites)
    print(f"Loaded {len(nodes)} SCATS sites")
    total_edges = sum(len(v) for v in edges.values()) // 2
    print(f"Created {total_edges} undirected edges (max {MAX_EDGE_KM} km)")
    print("\nSample edges from site 2000:")
    for nb, dist in edges.get(2000, []):
        print(f"  2000 -> {nb}  {dist:.3f} km")