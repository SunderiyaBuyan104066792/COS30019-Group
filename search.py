import sys
import math
import heapq
import random
import time
from collections import deque

#Parses input file to extract and format nodes, edges, origin, and destinations. 
def parse_file(filename):
    nodes = {}
    edges = {}
    origin = None
    destinations = set()
    landmarks = []

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

            if content == "Landmarks:":
                section = "landmarks" 
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

            elif section == "landmarks":
                parts = content.split(";")
                for part in parts:
                    part = part.strip()
                    if part != "":
                        landmarks.append(int(part))

# Sorts each neighbours by destination node ID 
    for from_node in edges:
        edges[from_node] = sorted(edges[from_node], key=lambda item: item[0])

    return nodes, edges, origin, destinations, landmarks

#Search node in tree
class Node:
    def __init__(self, state, parent=None, path_cost=0, depth=0, created_order=0):
        self.state = state
        self.parent = parent
        self.path_cost = path_cost
        self.depth = depth
        self.created_order = created_order

#Constructs path by tracing from goal to root. Returns path in correct order
def build_path(goal_node):
    path = []
    current = goal_node

    while current is not None:
        path.append(current.state)
        current = current.parent

    path.reverse()
    return path

#Calculates the straight line distance between two (x, y) coordinate points
def straight_line_distance(point_a, point_b):
    dx = point_b[0] - point_a[0]
    dy = point_b[1] - point_a[1]
    return math.sqrt(dx ** 2 + dy ** 2)

#Computes the straight-line distance from a state to the nearest destination using the calculation
def heuristic_distance(state, nodes, destinations):
    current_point = nodes[state]
    best_distance = float("inf")

    for goal in destinations:
        goal_point = nodes[goal]
        distance = straight_line_distance(current_point, goal_point)

        if distance < best_distance:
            best_distance = distance

    return best_distance

# Search algorithm: BFS
def breadth_first_search(nodes,edges, origin, destinations):
    if not origin or not destinations or origin not in nodes:
        return None, 0

    nodes_created = 0
    root = Node(origin, None, 0, 0, nodes_created)
    nodes_created += 1

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

# Search algorithm: DFS
def depth_first_search(nodes, edges, origin, destinations):
    if not origin or not destinations or origin not in nodes:
        return None, 0

    nodes_created = 0
    root = Node(origin, None, 0, 0, nodes_created)
    nodes_created += 1

    frontier = [root]
    visited = set()

    while frontier:
        current = frontier.pop()

        if current.state in visited:
            continue
        visited.add(current.state)

        if current.state in destinations:
            return current, nodes_created

        neighbours = edges.get(current.state, [])
        
        for next_state, _ in reversed(neighbours):
            if next_state not in visited:
                child = Node(next_state, current, 0, current.depth + 1, nodes_created)
                nodes_created += 1

                frontier.append(child)

    return None, nodes_created


# Search algorithm: GBFS
def greedy_best_first_search(nodes, edges, origin, destinations):
    if not origin or not destinations or origin not in nodes:
        return None, 0

    nodes_created = 0
    root = Node(origin, None, 0, 0, nodes_created)
    nodes_created += 1

    if root.state in destinations:
        return root, nodes_created

    frontier = []
    visited = set()

    root_h = heuristic_distance(origin, nodes, destinations)
    heapq.heappush(frontier, (root_h, origin, root.created_order, root))

    while frontier:
        current = heapq.heappop(frontier)[3]

        if current.state in visited:
            continue
        visited.add(current.state)

        if current.state in destinations:
            return current, nodes_created

        neighbours = edges.get(current.state, [])

        for next_state, edge_cost in neighbours:
            if next_state not in visited:
                child = Node(
                    next_state,
                    current,
                    current.path_cost + edge_cost,
                    current.depth + 1,
                    nodes_created
                )
                nodes_created += 1

                h = heuristic_distance(next_state, nodes, destinations)
                heapq.heappush(frontier, (h, next_state, child.created_order, child))

    return None, nodes_created

# Search algorithm: A Star
def astar_search(nodes, edges, origin, destinations):
    if not origin or not destinations or origin not in nodes:
        return None, 0

    nodes_created = 0
    counter = 0
    root = Node(origin, None, 0, 0, nodes_created)
    nodes_created += 1

    root_h = heuristic_distance(origin, nodes, destinations)

    # (f_cost, node_id, insertion_counter, node)
    frontier = [(root.path_cost + root_h, root.state, counter, root)]
    best_g = {origin: 0}

    while frontier:
        _, _, _, current = heapq.heappop(frontier)

        if current.path_cost > best_g.get(current.state, float("inf")):
            continue

        if current.state in destinations:
            return current, nodes_created
        
        neighbours = edges.get(current.state, [])

        for next_state, edge_cost in neighbours:
            new_g = current.path_cost + edge_cost

            if new_g < best_g.get(next_state, float("inf")):
                best_g[next_state] = new_g
                counter += 1

                child = Node(
                    next_state,
                    current,
                    new_g,
                    current.depth + 1,
                    counter
                )
                nodes_created += 1

                h = heuristic_distance(next_state, nodes, destinations)
                f = new_g + h

                heapq.heappush(frontier, (f, next_state, child.created_order, child))

    return None, nodes_created

# Search algorithm: CUS1 DLS
def depth_limited_search(nodes, edges, origin, destinations, limit):
    if not origin or not destinations or origin not in nodes:
        return None, 0

    CUTOFF = object()
    nodes_created = 0
    root = Node(origin, None, 0, 0, nodes_created)
    nodes_created += 1

    if root.state in destinations:
        return root, nodes_created

    def recursive_dls(current, visited, depth):
        nonlocal nodes_created

        if current.state in destinations:
            return current

        if depth == 0:
            return CUTOFF

        neighbours = edges.get(current.state, [])
        
        cutoff_occurred = False

        for next_state, _ in neighbours:
            if next_state in visited:
                continue

            child = Node(next_state, current, 0, current.depth + 1, nodes_created)
            nodes_created += 1

            visited.add(next_state)
            result = recursive_dls(child, visited, depth - 1)

            if result is CUTOFF:
                cutoff_occurred = True
            elif result is not None:
                return result

            visited.remove(next_state)

        if cutoff_occurred:
            return CUTOFF

        return None

    result = recursive_dls(root, {origin}, limit)

    if result is CUTOFF:
        return "CUTOFF", nodes_created
    if result is None:
        return None, nodes_created
    return result, nodes_created


# CUS 2: Helpers

#Computes an admissible heuristic using the ALT (A*, Landmarks, Triangle inequality) method.
def landmark_heuristic(state, destinations, landmarks, LM_Table):
    goals = []
    # to check for any of the valid destinationsz
    for goal in destinations:
        best_distance = float("-inf")
        for L in landmarks:
            if state not in LM_Table[L] or goal not in LM_Table[L]:
                continue
            dist = abs(LM_Table[L][state] - LM_Table[L][goal])
            best_distance = max(best_distance, dist)
        if best_distance != float("-inf"):
            goals.append(best_distance)

    if not goals:
        return 0

    return min(goals)

#Precomputes shortest distances from each landmark to all reachable nodes. 
#Returns a dictionary mapping each landmark to its distance dictionary.
def create_landmark_table(landmarks, edges):
    LM_Table = {}

    for L in landmarks:
        #value here is node to Landmark
        LM_Table[L] = (dijkstra_calc(edges, L))
    return LM_Table

#Dijkstra's algorithm from a landmark node and return a dictionary of shortest distances
def dijkstra_calc(edges, landmark):
    #same as A* but with h(n) = 0 and returns shortest distance instead of path to goal node
    nodes_created = 1
    counter = 0

    root = Node(landmark, None, 0, 0, counter)
        # (f_cost, node_id, insertion_counter, node)
    frontier = [(root.path_cost, root.state, counter, root)]
    best_g = {landmark: 0}
    landmark_dist = {}
    while frontier:
        _, _, _, current = heapq.heappop(frontier)

        if current.path_cost > best_g.get(current.state, float("inf")):
            continue
        #saves distance to landmark for the current state into a dictionary
        landmark_dist[current.state] = current.path_cost
        
        neighbours = edges.get(current.state, [])

        for next_state, edge_cost in neighbours:
            new_g = current.path_cost + edge_cost

            if new_g < best_g.get(next_state, float("inf")):
                best_g[next_state] = new_g
                counter += 1

                child = Node(
                    next_state,
                    current,
                    new_g,
                    current.depth + 1,
                    counter
                )
                nodes_created += 1

                f = new_g

                heapq.heappush(frontier, (f, next_state, child.created_order, child))
    return landmark_dist

#Search Algorithm: CUS2 ALT
def a_landmark_triangle_inequality_search(nodes, edges, origin, destinations, landmarks=None):
    if not origin or not destinations or origin not in nodes:
        return None, 0
    

    nodes_created = 0
    counter = 0

    root = Node(origin, None, 0, 0, nodes_created)
    nodes_created += 1
    
    if not landmarks:
        #create random landmarks based on given nodes with a minimum of 2 landmarks and a max of 16
        k = min(16, max(2, int(len(nodes) ** 0.5)))
        k = min(k, len(nodes))
        landmarks = random.sample(list(nodes.keys()), k)

    if root.state in destinations:
        return root, nodes_created
    
    #creates a dictionary contain all distances for all nodes from all landmarks
    landmark_table = create_landmark_table(landmarks, edges)
    root_h = landmark_heuristic(origin,destinations,landmarks,landmark_table)
    
    # (f_cost, node_id, insertion_counter, node)
    frontier = [(root.path_cost + root_h, root.state, counter, root)]
    best_g = {origin: 0}

    while frontier:
        _, _, _, current = heapq.heappop(frontier)

        if current.path_cost > best_g.get(current.state, float("inf")):
            continue

        if current.state in destinations:
            return current, nodes_created
        
        neighbours = edges.get(current.state, [])

        for next_state, edge_cost in neighbours:
            new_g = current.path_cost + edge_cost

            if new_g < best_g.get(next_state, float("inf")):
                best_g[next_state] = new_g

                child = Node(
                    next_state,
                    current,
                    new_g,
                    current.depth + 1,
                    nodes_created
                )
                nodes_created += 1
                counter += 1

                h = landmark_heuristic(next_state, destinations, landmarks, landmark_table)
                f = new_g + h

                heapq.heappush(frontier, (f, next_state, child.created_order, child))

    return None, nodes_created


#Prints the search result: goal reached, nodes created, and path.
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


def main():
    start = time.perf_counter()
    if len(sys.argv) < 3:
        print("Usage: python search.py <filename> <method> [limit]")
        sys.exit()

    filename = sys.argv[1]
    method = sys.argv[2].upper()

    nodes, edges, origin, destinations, landmarks = parse_file(filename)

    if method == "BFS":
        result_node, nodes_created = breadth_first_search(nodes, edges, origin, destinations)
    elif method == "DFS":
        result_node, nodes_created = depth_first_search(nodes,edges, origin, destinations)
    elif method == "GBFS":
        result_node, nodes_created = greedy_best_first_search(nodes, edges, origin, destinations)
    elif  method == "AS":
        result_node, nodes_created = astar_search(nodes, edges, origin, destinations)
    elif method == "DLS":
        if len(sys.argv) < 4:
            print("For DLS specify limit: python search.py <filename> DLS <limit>")
            sys.exit()

        limit = int(sys.argv[3])
        result_node, nodes_created = depth_limited_search(nodes, edges, origin, destinations, limit)
    elif  method == "ALT":
        result_node, nodes_created = a_landmark_triangle_inequality_search(nodes, edges, origin, destinations, landmarks)
    else:
        print("Please choose from BFS, DFS, GBFS, AS, DLS <limit>, ALT")
        return

    print_result(filename, method, result_node, nodes_created)
    end = time.perf_counter()
    print(f"Elapsed time: {end - start:.6f} seconds")


if __name__ == "__main__":
    main()
