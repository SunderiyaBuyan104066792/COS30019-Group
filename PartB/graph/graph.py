import math
import pandas as pd
from .node import Node
from .edge import Edge


class Graph:

    def __init__(self):
        self.nodes = {}
        self.edges = []

    def build(self):
        self.load_nodes()
        self.build_edges()

    def load_nodes(self):
        meta = pd.read_csv('graph/site_road_data/road_data.csv')
        for _, row in meta.iterrows():
            roads = set(row['roads'].split('|')) if pd.notna(row['roads']) else set()
            self.nodes[int(row['scats_num'])] = Node(
                scats_num = int(row['scats_num']),
                lat = row['latitude'],
                lon  = row['longitude'],
                roads = roads
            )

    def build_edges(self):
        sites = list(self.nodes.values())
        for i in range(len(sites)):
            for j in range(i + 1, len(sites)):
                a, b = sites[i], sites[j]
                shared = a.roads & b.roads
                if shared:
                    dist = self.haversine(a.lat, a.lon, b.lat, b.lon)
                    road = list(shared)[0]
                    self.edges.append(Edge(a, b, road, dist))
                    self.edges.append(Edge(b, a, road, dist))

    def get_neighbours(self, scats_num):
        return [e for e in self.edges if e.from_node.scats_num == scats_num]

    def get_node(self, scats_num):
        return self.nodes.get(scats_num)

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2): #Distance in km between two lat/lon points.
        R  = 6371
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat / 2) ** 2
                + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2)
        return round(R * 2 * math.asin(math.sqrt(a)), 4)

    def __repr__(self):
        return f'Graph({len(self.nodes)} nodes, {len(self.edges)} edges)'


def main():
    g = Graph()
    g.build()
    print(g)
    print()

if __name__ == '__main__':
    main()