"""
pathfinder.py
=============
Integrates Part A search algorithms with the TBRGS system.

Edge costs are dynamic travel times based on ML-predicted traffic flow.
Arrival time propagates through each node so each edge reflects actual
traffic conditions when the vehicle arrives.

Supports all 6 search algorithms from Part A.
Returns top-k routes using Yen's k-shortest paths algorithm.

Usage:
    from pathfinder import find_routes
    from datetime import datetime

    routes = find_routes(
        origin         = 2000,
        destination    = 3002,
        departure_time = datetime(2006, 11, 4, 8, 0),
        model_name     = 'lstm',
        method         = 'AS',
        k              = 5
    )
"""

import os
import heapq
import math
import random
from datetime import datetime, timedelta
from collections import deque

from graph.graph import Graph
from predict import predict_november
from graph.traveltime import travel_time


# ── Node ──────────────────────────────────────────────────────────────────────

class Node:
    def __init__(self, state, parent=None, path_cost=0,
                 arrival_time=None, depth=0, created_order=0):
        self.state         = state
        self.parent        = parent
        self.path_cost     = path_cost       # total seconds so far
        self.arrival_time  = arrival_time    # datetime at this node
        self.depth         = depth
        self.created_order = created_order


def build_path(goal_node):
    """Trace from goal back to root."""
    path, current = [], goal_node
    while current:
        path.append(current.state)
        current = current.parent
    path.reverse()
    return path


# ── Helpers ───────────────────────────────────────────────────────────────────

def haversine(lat1, lon1, lat2, lon2):
    R    = 6371
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a    = (math.sin(dlat / 2) ** 2
            + math.cos(math.radians(lat1))
            * math.cos(math.radians(lat2))
            * math.sin(dlon / 2) ** 2)
    return R * 2 * math.asin(math.sqrt(a))


def heuristic(state, destinations, graph):
    """
    Admissible heuristic — straight-line distance at speed limit (60 km/h).
    Never overestimates since actual travel time >= free-flow time.
    Returns seconds.
    """
    node = graph.get_node(state)
    if node is None:
        return 0
    best = float('inf')
    for dest in destinations:
        dest_node = graph.get_node(dest)
        if dest_node is None:
            continue
        dist    = haversine(node.lat, node.lon, dest_node.lat, dest_node.lon)
        min_time = (dist / 60) * 3600
        if min_time < best:
            best = min_time
    return best


def edge_cost(edge, arrival_time, model_name):
    """
    Dynamic travel time for an edge based on predicted flow at arrival_time.
    Flow is taken from the starting SCATS site per the assignment spec.
    """
    flow = predict_november(
        site       = edge.from_node.scats_num,
        query_time = arrival_time,
        model_name = model_name
    )
    if flow is None:
        flow = 0  # fallback to free flow
    return travel_time(edge.distance, flow)


def get_neighbours(graph, state, excluded_edges):
    """Return edges from state, skipping any excluded (from, to) pairs."""
    return [
        e for e in graph.get_neighbours(state)
        if (e.from_node.scats_num, e.to_node.scats_num) not in excluded_edges
    ]


# ── Search algorithms ─────────────────────────────────────────────────────────

def bfs(graph, origin, destinations, departure_time, model_name, excluded_edges):
    """Breadth-First Search."""
    nodes_created = 0
    root = Node(origin, arrival_time=departure_time, created_order=nodes_created)
    nodes_created += 1

    frontier = deque([root])
    visited  = {origin}

    while frontier:
        current = frontier.popleft()
        if current.state in destinations:
            return current, nodes_created

        for edge in get_neighbours(graph, current.state, excluded_edges):
            if edge.to_node.scats_num not in visited:
                cost  = edge_cost(edge, current.arrival_time, model_name)
                child = Node(
                    state        = edge.to_node.scats_num,
                    parent       = current,
                    path_cost    = current.path_cost + cost,
                    arrival_time = current.arrival_time + timedelta(seconds=cost),
                    depth        = current.depth + 1,
                    created_order= nodes_created
                )
                nodes_created += 1
                visited.add(edge.to_node.scats_num)
                frontier.append(child)

    return None, nodes_created


def dfs(graph, origin, destinations, departure_time, model_name, excluded_edges):
    """Depth-First Search."""
    nodes_created = 0
    root    = Node(origin, arrival_time=departure_time, created_order=nodes_created)
    nodes_created += 1
    frontier = [root]
    visited  = set()

    while frontier:
        current = frontier.pop()
        if current.state in visited:
            continue
        visited.add(current.state)

        if current.state in destinations:
            return current, nodes_created

        for edge in reversed(get_neighbours(graph, current.state, excluded_edges)):
            if edge.to_node.scats_num not in visited:
                cost  = edge_cost(edge, current.arrival_time, model_name)
                child = Node(
                    state        = edge.to_node.scats_num,
                    parent       = current,
                    path_cost    = current.path_cost + cost,
                    arrival_time = current.arrival_time + timedelta(seconds=cost),
                    depth        = current.depth + 1,
                    created_order= nodes_created
                )
                nodes_created += 1
                frontier.append(child)

    return None, nodes_created


def gbfs(graph, origin, destinations, departure_time, model_name, excluded_edges):
    """Greedy Best-First Search."""
    nodes_created = 0
    counter = 0
    root    = Node(origin, arrival_time=departure_time, created_order=nodes_created)
    nodes_created += 1

    if origin in destinations:
        return root, nodes_created

    frontier = []
    visited  = set()
    h = heuristic(origin, destinations, graph)
    heapq.heappush(frontier, (h, origin, counter, root))

    while frontier:
        _, _, _, current = heapq.heappop(frontier)
        if current.state in visited:
            continue
        visited.add(current.state)

        if current.state in destinations:
            return current, nodes_created

        for edge in get_neighbours(graph, current.state, excluded_edges):
            if edge.to_node.scats_num not in visited:
                cost  = edge_cost(edge, current.arrival_time, model_name)
                child = Node(
                    state        = edge.to_node.scats_num,
                    parent       = current,
                    path_cost    = current.path_cost + cost,
                    arrival_time = current.arrival_time + timedelta(seconds=cost),
                    depth        = current.depth + 1,
                    created_order= nodes_created
                )
                nodes_created += 1
                counter += 1
                h = heuristic(edge.to_node.scats_num, destinations, graph)
                heapq.heappush(frontier, (h, edge.to_node.scats_num,
                                          child.created_order, child))

    return None, nodes_created


def astar(graph, origin, destinations, departure_time, model_name, excluded_edges):
    """A* Search."""
    nodes_created = 0
    counter = 0
    root    = Node(origin, arrival_time=departure_time, created_order=nodes_created)
    nodes_created += 1

    if origin in destinations:
        return root, nodes_created

    h = heuristic(origin, destinations, graph)
    frontier = [(root.path_cost + h, origin, counter, root)]
    best_g   = {origin: 0}

    while frontier:
        _, _, _, current = heapq.heappop(frontier)

        if current.path_cost > best_g.get(current.state, float('inf')):
            continue
        if current.state in destinations:
            return current, nodes_created

        for edge in get_neighbours(graph, current.state, excluded_edges):
            cost  = edge_cost(edge, current.arrival_time, model_name)
            new_g = current.path_cost + cost

            if new_g < best_g.get(edge.to_node.scats_num, float('inf')):
                best_g[edge.to_node.scats_num] = new_g
                counter += 1
                child = Node(
                    state        = edge.to_node.scats_num,
                    parent       = current,
                    path_cost    = new_g,
                    arrival_time = current.arrival_time + timedelta(seconds=cost),
                    depth        = current.depth + 1,
                    created_order= counter
                )
                nodes_created += 1
                h = heuristic(edge.to_node.scats_num, destinations, graph)
                heapq.heappush(frontier, (new_g + h, edge.to_node.scats_num,
                                          child.created_order, child))

    return None, nodes_created


def dls(graph, origin, destinations, departure_time, model_name,
        excluded_edges, limit=10):
    """Depth-Limited Search (CUS1)."""
    CUTOFF = object()
    nodes_created = 0
    root = Node(origin, arrival_time=departure_time, created_order=nodes_created)
    nodes_created += 1

    if origin in destinations:
        return root, nodes_created

    def recursive_dls(current, visited, depth):
        nonlocal nodes_created
        if current.state in destinations:
            return current
        if depth == 0:
            return CUTOFF
        cutoff_occurred = False
        for edge in get_neighbours(graph, current.state, excluded_edges):
            nxt = edge.to_node.scats_num
            if nxt in visited:
                continue
            cost  = edge_cost(edge, current.arrival_time, model_name)
            child = Node(
                state        = nxt,
                parent       = current,
                path_cost    = current.path_cost + cost,
                arrival_time = current.arrival_time + timedelta(seconds=cost),
                depth        = current.depth + 1,
                created_order= nodes_created
            )
            nodes_created += 1
            visited.add(nxt)
            result = recursive_dls(child, visited, depth - 1)
            if result is CUTOFF:
                cutoff_occurred = True
            elif result is not None:
                return result
            visited.remove(nxt)
        return CUTOFF if cutoff_occurred else None

    result = recursive_dls(root, {origin}, limit)
    if result is CUTOFF or result is None:
        return None, nodes_created
    return result, nodes_created


def alt(graph, origin, destinations, departure_time, model_name, excluded_edges):
    """ALT Search — A* with Landmark Triangle inequality heuristic (CUS2)."""
    nodes_created = 0
    counter = 0

    # Pick random landmarks
    all_sites = list(graph.nodes.keys())
    k = min(16, max(2, int(len(all_sites) ** 0.5)))
    landmarks = random.sample(all_sites, k)

    # Precompute landmark distances using Dijkstra
    lm_table = {L: _dijkstra(graph, L, model_name, departure_time)
                for L in landmarks}

    def lm_heuristic(state, dests):
        best = float('-inf')
        for dest in dests:
            for L in landmarks:
                if state not in lm_table[L] or dest not in lm_table[L]:
                    continue
                best = max(best, abs(lm_table[L][state] - lm_table[L][dest]))
        return max(0, best)

    root = Node(origin, arrival_time=departure_time, created_order=nodes_created)
    nodes_created += 1

    if origin in destinations:
        return root, nodes_created

    h = lm_heuristic(origin, destinations)
    frontier = [(root.path_cost + h, origin, counter, root)]
    best_g   = {origin: 0}

    while frontier:
        _, _, _, current = heapq.heappop(frontier)
        if current.path_cost > best_g.get(current.state, float('inf')):
            continue
        if current.state in destinations:
            return current, nodes_created

        for edge in get_neighbours(graph, current.state, excluded_edges):
            cost  = edge_cost(edge, current.arrival_time, model_name)
            new_g = current.path_cost + cost
            if new_g < best_g.get(edge.to_node.scats_num, float('inf')):
                best_g[edge.to_node.scats_num] = new_g
                counter += 1
                child = Node(
                    state        = edge.to_node.scats_num,
                    parent       = current,
                    path_cost    = new_g,
                    arrival_time = current.arrival_time + timedelta(seconds=cost),
                    depth        = current.depth + 1,
                    created_order= counter
                )
                nodes_created += 1
                h = lm_heuristic(edge.to_node.scats_num, destinations)
                heapq.heappush(frontier, (new_g + h, edge.to_node.scats_num,
                                          child.created_order, child))

    return None, nodes_created


def _dijkstra(graph, start, model_name, departure_time):
    """Dijkstra from a landmark — used by ALT heuristic."""
    frontier = [(0, start)]
    dist     = {start: 0}
    while frontier:
        d, state = heapq.heappop(frontier)
        if d > dist.get(state, float('inf')):
            continue
        for edge in graph.get_neighbours(state):
            cost = edge_cost(edge, departure_time, model_name)
            nd   = d + cost
            if nd < dist.get(edge.to_node.scats_num, float('inf')):
                dist[edge.to_node.scats_num] = nd
                heapq.heappush(frontier, (nd, edge.to_node.scats_num))
    return dist


# ── Search method dispatcher ──────────────────────────────────────────────────

SEARCH_METHODS = {
    'BFS' : bfs,
    'DFS' : dfs,
    'GBFS': gbfs,
    'AS'  : astar,
    'CUS1': dls,
    'CUS2': alt,
}


# ── Yen's k-shortest paths ────────────────────────────────────────────────────

def _path_cost(graph, path, departure_time, model_name):
    """Replay a path with time-propagating edge costs. Returns (total_seconds, arrival_time)."""
    total = 0.0
    t = departure_time
    for i in range(len(path) - 1):
        edge = next(
            (e for e in graph.get_neighbours(path[i])
             if e.to_node.scats_num == path[i + 1]),
            None
        )
        if edge is None:
            return float('inf'), t
        cost = edge_cost(edge, t, model_name)
        total += cost
        t += timedelta(seconds=cost)
    return total, t


def yen_k_shortest(graph, origin, destination, departure_time,
                   model_name, method, k=5):
    """
    Find top-k fastest routes using Yen's k-shortest paths.
    Works with any search algorithm.

    # Arguments
        graph          : Graph, road network.
        origin         : int, SCATS origin site number.
        destination    : int, SCATS destination site number.
        departure_time : datetime, trip start time.
        model_name     : str, 'lstm', 'gru' or 'custom'.
        method         : str, 'BFS', 'DFS', 'GBFS', 'AS', 'CUS1', 'CUS2'.
        k              : int, number of routes to return.
    # Returns
        routes: list of dicts with 'path', 'cost_seconds', 'arrival_time'
    """
    search_fn   = SEARCH_METHODS[method.upper()]
    destinations = {destination}
    routes       = []
    candidates   = []

    # First route
    goal, _ = search_fn(graph, origin, destinations, departure_time,
                        model_name, set())
    if goal is None:
        return []

    routes.append({
        'path'         : build_path(goal),
        'cost_seconds' : goal.path_cost,
        'arrival_time' : goal.arrival_time
    })

    for _ in range(k - 1):
        last_path = routes[-1]['path']

        for i in range(len(last_path) - 1):
            spur_node = last_path[i]
            root_path = last_path[:i + 1]

            # Exclude edges used by all routes sharing root_path
            excluded = set()
            for route in routes:
                if route['path'][:i + 1] == root_path:
                    excluded.add((route['path'][i], route['path'][i + 1]))

            goal, _ = search_fn(graph, spur_node, destinations,
                                 departure_time, model_name, excluded)
            if goal is None:
                continue

            spur_path = build_path(goal)
            full_path = root_path[:-1] + spur_path

            total_cost, arrival = _path_cost(graph, full_path, departure_time, model_name)
            candidate = {
                'path'        : full_path,
                'cost_seconds': total_cost,
                'arrival_time': arrival
            }

            if candidate not in candidates and candidate not in routes:
                candidates.append(candidate)

        if not candidates:
            break

        candidates.sort(key=lambda x: x['cost_seconds'])
        routes.append(candidates.pop(0))

    return routes


# ── Public API ────────────────────────────────────────────────────────────────

def find_routes(origin, destination, departure_time,
                model_name='lstm', method='AS', k=5):
    """
    Find top-k fastest routes between two SCATS sites.

    # Arguments
        origin         : int, SCATS origin site number.
        destination    : int, SCATS destination site number.
        departure_time : datetime, November 2006 departure time.
        model_name     : str, 'lstm', 'gru' or 'custom'.
        method         : str, 'BFS', 'DFS', 'GBFS', 'AS', 'CUS1', 'CUS2'.
        k              : int, number of routes to return (default 5).
    # Returns
        routes: list of dicts with 'path', 'cost_seconds', 'arrival_time'
    """
    if method.upper() not in SEARCH_METHODS:
        raise ValueError(f'Unknown method {method}. Choose from: '
                         f'{list(SEARCH_METHODS.keys())}')

    graph = Graph()
    graph.build()
    return yen_k_shortest(graph, origin, destination, departure_time,
                          model_name, method, k)


# ── CLI entry point ───────────────────────────────────────────────────────────

def main():
    origin      = 2000
    destination = 3002
    depart      = datetime(2006, 11, 4, 8, 0, 0)  # November 2006
    model_name  = 'lstm'
    method      = 'AS'

    print(f'Finding top 5 routes: {origin} -> {destination}')
    print(f'Departure:  {depart.strftime("%Y-%m-%d %H:%M")}')
    print(f'Model:      {model_name.upper()}')
    print(f'Algorithm:  {method}')
    print()

    routes = find_routes(origin, destination, depart, model_name, method, k=5)

    if not routes:
        print('No routes found.')
        return

    for i, route in enumerate(routes, 1):
        mins = route['cost_seconds'] / 60
        print(f'Route {i}: {" -> ".join(str(s) for s in route["path"])}')
        print(f'  Travel time: {mins:.1f} min')
        print(f'  Arrive:      {route["arrival_time"].strftime("%H:%M:%S")}')
        print()


if __name__ == '__main__':
    main()