import heapq
import random
from datetime import datetime, timedelta
from collections import deque

from graph.graph import Graph
from predict import predict_november
from graph.traveltime import travel_time
from graph.search_node import SearchNode, build_path


def heuristic_distance(state, nodes, destinations):
    node = nodes.get(state)
    if node is None:
        return 0
    best = float('inf')
    for dest in destinations:
        dest_node = nodes.get(dest)
        if dest_node is None:
            continue
        dist = Graph.haversine(node.lat, node.lon, dest_node.lat, dest_node.lon)
        min_time = (dist / 60) * 3600
        if min_time < best:
            best = min_time
    return best


def edge_cost(edge, arrival_time, model_name):
    flow = predict_november(
        site=edge.from_node.scats_num,
        query_time=arrival_time,
        model_name=model_name
    )
    if flow is None:
        flow = 0
    return travel_time(edge.distance, flow)


def bfs(nodes, edges, origin, destinations):
    if not origin or not destinations or origin not in nodes:
        return None, 0

    nodes_created = 0
    root = SearchNode(state=origin, parent=None, path_cost=0,
                      depth=0, created_order=nodes_created)
    nodes_created += 1

    frontier = deque([root])
    visited = {origin}

    while frontier:
        current = frontier.popleft()

        if current.state in destinations:
            return current, nodes_created

        for next_state, _ in edges.get(current.state, []):
            if next_state not in visited:
                child = SearchNode(state=next_state, parent=current, path_cost=0,
                                   depth=current.depth + 1, created_order=nodes_created)
                nodes_created += 1
                visited.add(next_state)
                frontier.append(child)

    return None, nodes_created


def dfs(nodes, edges, origin, destinations):
    if not origin or not destinations or origin not in nodes:
        return None, 0

    nodes_created = 0
    root = SearchNode(state=origin, parent=None, path_cost=0,
                      depth=0, created_order=nodes_created)
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

        for next_state, _ in reversed(edges.get(current.state, [])):
            if next_state not in visited:
                child = SearchNode(state=next_state, parent=current, path_cost=0,
                                   depth=current.depth + 1, created_order=nodes_created)
                nodes_created += 1
                frontier.append(child)

    return None, nodes_created


def gbfs(nodes, edges, origin, destinations):
    if not origin or not destinations or origin not in nodes:
        return None, 0

    nodes_created = 0
    root = SearchNode(state=origin, parent=None, path_cost=0,
                      depth=0, created_order=nodes_created)
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

        for next_state, w in edges.get(current.state, []):
            if next_state not in visited:
                child = SearchNode(state=next_state, parent=current,
                                   path_cost=current.path_cost + w,
                                   depth=current.depth + 1, created_order=nodes_created)
                nodes_created += 1
                h = heuristic_distance(next_state, nodes, destinations)
                heapq.heappush(frontier, (h, next_state, child.created_order, child))

    return None, nodes_created


def astar(nodes, edges, origin, destinations):
    if not origin or not destinations or origin not in nodes:
        return None, 0

    nodes_created = 0
    counter = 0
    root = SearchNode(state=origin, parent=None, path_cost=0,
                      depth=0, created_order=nodes_created)
    nodes_created += 1

    root_h = heuristic_distance(origin, nodes, destinations)
    frontier = [(root.path_cost + root_h, root.state, counter, root)]
    best_g = {origin: 0}

    while frontier:
        _, _, _, current = heapq.heappop(frontier)

        if current.path_cost > best_g.get(current.state, float('inf')):
            continue
        if current.state in destinations:
            return current, nodes_created

        for next_state, w in edges.get(current.state, []):
            new_g = current.path_cost + w

            if new_g < best_g.get(next_state, float('inf')):
                best_g[next_state] = new_g
                counter += 1
                child = SearchNode(state=next_state, parent=current,
                                   path_cost=new_g,
                                   depth=current.depth + 1, created_order=counter)
                nodes_created += 1
                h = heuristic_distance(next_state, nodes, destinations)
                heapq.heappush(frontier, (new_g + h, next_state, child.created_order, child))

    return None, nodes_created


def dls(nodes, edges, origin, destinations, limit=10):
    """Depth-Limited Search (CUS1)."""
    if not origin or not destinations or origin not in nodes:
        return None, 0

    CUTOFF = object()
    nodes_created = 0
    root = SearchNode(state=origin, parent=None, path_cost=0,
                      depth=0, created_order=nodes_created)
    nodes_created += 1

    if root.state in destinations:
        return root, nodes_created

    def recursive_dls(current, visited, depth):
        nonlocal nodes_created
        if current.state in destinations:
            return current
        if depth == 0:
            return CUTOFF
        cutoff_occurred = False
        for next_state, _ in edges.get(current.state, []):
            if next_state in visited:
                continue
            child = SearchNode(state=next_state, parent=current, path_cost=0,
                               depth=current.depth + 1, created_order=nodes_created)
            nodes_created += 1
            visited.add(next_state)
            result = recursive_dls(child, visited, depth - 1)
            if result is CUTOFF:
                cutoff_occurred = True
            elif result is not None:
                return result
            visited.remove(next_state)
        return CUTOFF if cutoff_occurred else None

    result = recursive_dls(root, {origin}, limit)
    if result is CUTOFF or result is None:
        return None, nodes_created
    return result, nodes_created


def landmark_heuristic(state, destinations, landmarks, lm_table):
    goals = []
    for goal in destinations:
        best = float('-inf')
        for L in landmarks:
            if state not in lm_table[L] or goal not in lm_table[L]:
                continue
            best = max(best, abs(lm_table[L][state] - lm_table[L][goal]))
        if best != float('-inf'):
            goals.append(best)
    return min(goals) if goals else 0


def _dijkstra_calc(edges, landmark):
    counter = 0
    root = SearchNode(state=landmark, parent=None, path_cost=0,
                      depth=0, created_order=counter)
    frontier = [(0, landmark, counter, root)]
    best_g = {landmark: 0}
    dist = {}
    while frontier:
        _, _, _, current = heapq.heappop(frontier)
        if current.path_cost > best_g.get(current.state, float('inf')):
            continue
        dist[current.state] = current.path_cost
        for next_state, w in edges.get(current.state, []):
            new_g = current.path_cost + w
            if new_g < best_g.get(next_state, float('inf')):
                best_g[next_state] = new_g
                counter += 1
                child = SearchNode(state=next_state, parent=current,
                                   path_cost=new_g,
                                   depth=current.depth + 1, created_order=counter)
                heapq.heappush(frontier, (new_g, next_state, child.created_order, child))
    return dist


def _create_landmark_table(landmarks, edges):
    return {L: _dijkstra_calc(edges, L) for L in landmarks}


def alt(nodes, edges, origin, destinations, landmarks=None):
    """ALT Search — A* with Landmark Triangle Inequality heuristic (CUS2)."""
    if not origin or not destinations or origin not in nodes:
        return None, 0

    nodes_created = 0
    counter = 0
    root = SearchNode(state=origin, parent=None, path_cost=0,
                      depth=0, created_order=nodes_created)
    nodes_created += 1

    if not landmarks:
        k = min(16, max(2, int(len(nodes) ** 0.5)))
        landmarks = random.sample(list(nodes.keys()), min(k, len(nodes)))

    if root.state in destinations:
        return root, nodes_created

    lm_table = _create_landmark_table(landmarks, edges)
    root_h = landmark_heuristic(origin, destinations, landmarks, lm_table)

    frontier = [(root.path_cost + root_h, root.state, counter, root)]
    best_g = {origin: 0}

    while frontier:
        _, _, _, current = heapq.heappop(frontier)

        if current.path_cost > best_g.get(current.state, float('inf')):
            continue
        if current.state in destinations:
            return current, nodes_created

        for next_state, w in edges.get(current.state, []):
            new_g = current.path_cost + w
            if new_g < best_g.get(next_state, float('inf')):
                best_g[next_state] = new_g
                child = SearchNode(state=next_state, parent=current,
                                   path_cost=new_g,
                                   depth=current.depth + 1, created_order=nodes_created)
                nodes_created += 1
                counter += 1
                h = landmark_heuristic(next_state, destinations, landmarks, lm_table)
                heapq.heappush(frontier, (new_g + h, next_state, child.created_order, child))

    return None, nodes_created


SEARCH_METHODS = {
    'BFS': bfs,
    'DFS': dfs,
    'GBFS': gbfs,
    'AS': astar,
    'DLS': dls,
    'ALT': alt,
}


def _build_edges_dict(graph, departure_time, model_name):
    edges = {}
    for edge in graph.edges:
        cost = edge_cost(edge, departure_time, model_name)
        edges.setdefault(edge.from_node.scats_num, []).append(
            (edge.to_node.scats_num, cost)
        )
    return edges


def _path_cost(graph, path, departure_time, model_name):
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


def find_routes(origin, destination, departure_time,
                model_name='lstm', method='AS', k=5):
    if method.upper() not in SEARCH_METHODS:
        raise ValueError(f'Unknown method "{method}". Choose from: {list(SEARCH_METHODS)}')

    graph = Graph()
    graph.build()
    edges_dict = _build_edges_dict(graph, departure_time, model_name)
    nodes_dict = graph.nodes
    search_fn = SEARCH_METHODS[method.upper()]

    goal, _ = search_fn(nodes_dict, edges_dict, origin, {destination})
    if goal is None:
        return []

    A = [build_path(goal)]
    B_heap = []
    seen_B = set()

    for _ in range(1, k):
        prev_path = A[-1]

        for i in range(len(prev_path) - 1):
            spur_node = prev_path[i]
            root_path = prev_path[:i + 1]
            blocked_next = {pa[i + 1] for pa in A
                            if pa[:i + 1] == root_path and len(pa) > i + 1}
            blocked_nodes = set(root_path[:-1])

            spur_edges = {
                node: [(n, w) for n, w in nbrs
                       if n not in blocked_nodes
                       and not (node == spur_node and n in blocked_next)]
                for node, nbrs in edges_dict.items()
                if node not in blocked_nodes
            }

            spur_goal, _ = search_fn(nodes_dict, spur_edges, spur_node, {destination})
            if spur_goal is None:
                continue

            full_path = root_path[:-1] + build_path(spur_goal)
            key = tuple(full_path)
            if key not in seen_B and full_path not in A:
                seen_B.add(key)
                cost, _ = _path_cost(graph, full_path, departure_time, model_name)
                heapq.heappush(B_heap, (cost, full_path))

        if not B_heap:
            break
        _, best = heapq.heappop(B_heap)
        A.append(best)

    routes = []
    for path in A:
        cost_sec, arrival = _path_cost(graph, path, departure_time, model_name)
        routes.append({
            'path': path,
            'cost_seconds': cost_sec,
            'arrival_time': arrival,
        })
    routes.sort(key=lambda r: r['cost_seconds'])
    return routes


def main():
    origin = 3180
    destination = 4262
    depart = datetime(2006, 11, 4, 11, 30, 0)
    model_name = 'lstm'
    method = 'AS'

    print(f'Finding top 5 routes: {origin} -> {destination}')
    print(f'Departure:  {depart.strftime("%Y-%m-%d %H:%M")}')
    print(f'Model:      {model_name.upper()}')
    print(f'Algorithm:  {method}')
    print()

    routes = find_routes(origin, destination, depart, model_name, method=method, k=5)

    if not routes:
        print('No routes found.')
        return

    for i, route in enumerate(routes, 1):
        total_sec = int(route['cost_seconds'])
        mins, secs = divmod(total_sec, 60)
        print(f'Route {i}: {" -> ".join(str(s) for s in route["path"])}')
        print(f'  Travel time: {mins} min {secs} sec')
        print(f'  Arrive:      {route["arrival_time"].strftime("%H:%M:%S")}')
        print()


if __name__ == '__main__':
    main()
