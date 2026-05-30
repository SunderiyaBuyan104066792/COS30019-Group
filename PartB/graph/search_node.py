class SearchNode:
    def __init__(self, state, parent=None, path_cost=0,
                 arrival_time=None, depth=0, created_order=0):
        self.state = state
        self.parent = parent
        self.path_cost = path_cost
        self.arrival_time = arrival_time
        self.depth = depth
        self.created_order = created_order


def build_path(goal_node):
    path, current = [], goal_node
    while current:
        path.append(current.state)
        current = current.parent
    path.reverse()
    return path
