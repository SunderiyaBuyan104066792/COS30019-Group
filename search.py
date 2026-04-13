import sys
import math
import heapq
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
                    destinations.add(int(part.strip()))

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


def straight_line_distance(point_a, point_b):
    dx = point_b[0] - point_a[0]
    dy = point_b[1] - point_a[1]
    return math.sqrt(dx ** 2 + dy ** 2)


def hn(state, nodes, destinations):
    current_point = nodes[state]
    best_distance = float("inf")

    for goal in destinations:
        goal_point = nodes[goal]
        distance = straight_line_distance(current_point, goal_point)

        if distance < best_distance:
            best_distance = distance

    return best_distance


def greedy_best_first_search(nodes, edges, origin, destinations):
    nodes_created = 1
    root = Node(origin, None, 0, 0, 0)

    if root.state in destinations:
        return root, nodes_created

    frontier = []
    root_h = hn(origin, nodes, destinations)
    heapq.heappush(frontier, (root_h, origin, root.created_order, root))

    while frontier:
        current = heapq.heappop(frontier)[3]

        if current.state in destinations:
            return current, nodes_created

        for next_state, edge_cost in edges.get(current.state, []):
            child = Node(
                next_state,
                current,
                current.path_cost + edge_cost,
                current.depth + 1,
                nodes_created
            )
            nodes_created += 1

            h = hn(next_state, nodes, destinations)
            heapq.heappush(frontier, (h, next_state, child.created_order, child))

    return None, nodes_created


def print_result(filename, method, result_node, nodes_created):
    print(filename, method)

    if result_node is None:
        print("None", nodes_created)
        print("No path found")
        return

    path = build_path(result_node)
    print(result_node.state, nodes_created)
    print(" ".join(str(state) for state in path))


def main():
    if len(sys.argv) != 3:
        print("Usage: python search.py <filename> <method>")
        sys.exit()

    filename = sys.argv[1]
    method = sys.argv[2].upper()

    nodes, edges, origin, destinations = parse_file(filename)

    #DELETE BEFORE SUBMISSION - See how i formatted it
    print(nodes)
    print(edges)
    print(origin)
    print(destinations)

    if method == "GBFS":
        result_node, nodes_created = greedy_best_first_search(nodes, edges, origin, destinations)
    #ADD yours here
    else:
        print("Please choose from BFS, DFS, GBFS, A*, CUS1, CUS2")
        return

    print_result(filename, method, result_node, nodes_created)


if __name__ == "__main__":
    main()