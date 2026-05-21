class Node:

    def __init__(self, scats_num, lat, lon, roads):
        self.scats_num = scats_num
        self.lat       = lat
        self.lon       = lon
        self.roads     = roads

    def __repr__(self):
        return f'Node({self.scats_num})'