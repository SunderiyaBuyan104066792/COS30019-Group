import sys
import math
import heapq
import random
from collections import deque


def parse_file(filename):
    nodes = {}
    edges = {}
    origin = None
    destinations = set()

    section = ""

    with open(filename) as f:
        for line in f:
            content = line.strip()

            if content == "":
                continue

            if content == "Nodes:":
                section = "nodes"
                continue
            if content == "Edges:":
                section = "edges"
                continue
            if content == "Origin:":
                section = "origin"
                continue
            if content == "Destinations:":
                section = "destinations"
                continue

            if section == "nodes":
                n_id, coords = content.split(":")
                x, y = coords.strip().strip("()").split(",")
                nodes[int(n_id.strip())] = (int(x.strip()), int(y.strip()))

            elif section == "edges":
                n_edges, e_cost = content.split(":")
                cost = int(e_cost.strip())

                n_from, n_to = n_edges.strip().strip("()").split(",")
                from_node = int(n_from.strip())
                to_node = int(n_to.strip())

                if from_node in edges:
                    edges[from_node].append((to_node, cost))
                else:
                    edges[from_node] = [(to_node, cost)]

            elif section == "origin":
                origin = int(content)

            elif section == "destinations":
                parts = content.split(";")
                for part in parts:
                    part = part.strip()
                    if part != "":
                        destinations.add(int(part))

    for from_node in edges:
        edges[from_node] = sorted(edges[from_node], key=lambda item: item[0])

    return nodes, edges, origin, destinations


class Node:
    def __init__(self, state, parent=None, path_cost=0, depth=0, created_order=0):
        self.state = state
        self.parent = parent
        self.path_cost = path_cost
        self.depth = depth
        self.created_order = created_order


def build_path(goal_node):
    path = []
    current = goal_node

    while current is not None:
        path.append(current.state)
        current = current.parent

    path.reverse()
    return path



def breadth_first_search(edges, origin, destinations):
    nodes_created = 1
    root = Node(origin)

    if root.state in destinations:
        return root, nodes_created
    
    if root.state not in edges:
        return None,0

    frontier = deque([root])
    visited = {origin}

    while frontier:
        current = frontier.popleft()

        if current.state in destinations:
            return current, nodes_created

        neighbours = edges.get(current.state, [])

        for next_state, _ in neighbours:
            if next_state not in visited:
                child = Node(next_state, current, 0, current.depth + 1, nodes_created)
                nodes_created += 1

                visited.add(next_state)
                frontier.append(child)

    return None, nodes_created



def print_result(filename, method, result_node, nodes_created):
    print(filename, method)

    #For DLS Cutoff result 
    if result_node == "CUTOFF":
        print("CUTOFF", nodes_created)
        print("DLS: Depth limit reached")
        return

    if result_node is None:
        print("None", nodes_created)
        print("No path found")
        return

    path = build_path(result_node)
    print(result_node.state, nodes_created)
    print(" ".join(str(state) for state in path))


def BFS_tests():
    #test 1 tests if BFS only searches in an area which has connected edges
    test_graph_1 = {1: [(2,3)], 2:[(3,2)], 3:[(1,4)], 4:[(4,1)]}
    test_origin_1 = 2
    test_dest_1 = {3}
    result, nodes_created = breadth_first_search(test_graph_1,test_origin_1,test_dest_1)
    path = build_path(result)
    if path == [2,3]:
        print("test 1 passed")
    else:
        print("test 2 failed")
    
    #test 2 if BFS returns an error when given an start node that is not in the graph
    test_graph_2 = {1: [(2,3)], 2:[(3,2)], 3:[(1,4)], 4:[(4,1)]}
    test_origin_2 = 5
    test_dest_2 = {3}
    result, nodes_created = breadth_first_search(test_graph_2,test_origin_2,test_dest_2)
    if result == None and nodes_created ==  0:
        print("test 2 passed")
    else:
        print("test 2 failed")

    #test 3 if BFS returns an error when given an start node that is not in the graph
    test_graph_3 = {1: [(2,3)], 2:[(3,2)], 3:[(1,4)], 4:[(4,1)]}
    test_origin_3 = 2
    test_dest_3 = {2}
    result, nodes_created = breadth_first_search(test_graph_3,test_origin_3,test_dest_3)
    path = build_path(result)
    if path == [2] and nodes_created == 1:
        print("test 3 passed")
    else:
        print("test 3 failed")
    

def main():
    if len(sys.argv) < 3:
        print("Usage: python search.py <filename> <method> [limit]")
        sys.exit()

    filename = sys.argv[1]
    method = sys.argv[2].upper()

    nodes, edges, origin, destinations = parse_file(filename)

    if method == "BFS":
        result_node, nodes_created = breadth_first_search(edges, origin, destinations)
    # elif method == "DFS":
    #     result_node, nodes_created = depth_first_search(edges, origin, destinations)
    # elif method == "GBFS":
    #     result_node, nodes_created = greedy_best_first_search(nodes, edges, origin, destinations)
    # elif  method == "AS":
    #     result_node, nodes_created = astar_search(nodes, edges, origin, destinations)
    # elif method == "DLS":
    #     if len(sys.argv) < 4:
    #         print("For DLS specify limit: python search.py <filename> DLS <limit>")
    #         sys.exit()

    #     limit = int(sys.argv[3])
    #     result_node, nodes_created = depth_limited_search(origin, destinations, edges, limit)
    # elif  method == "ALT":
    #     result_node, nodes_created = a_landmark_triangle_inequality_search(nodes, edges, origin, destinations)
    # else:
    #     print("Please choose from BFS, DFS, GBFS, AS, DLS <limit>, ALT")
    #     return

    print_result(filename, method, result_node, nodes_created)


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1].upper() == "TEST":
        BFS_tests()
    else:
        main()
