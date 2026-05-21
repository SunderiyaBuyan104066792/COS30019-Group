class Edge:
 
    def __init__(self, from_node, to_node, road, distance):
        self.from_node = from_node
        self.to_node   = to_node
        self.road      = road
        self.distance  = distance

    def __repr__(self):
        return f'Edge({self.from_node.scats_num} -> {self.to_node.scats_num} | {self.road} | {self.distance:.4f} km)'

