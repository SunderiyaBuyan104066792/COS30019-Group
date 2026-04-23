import sys
import heapq
import math


def parse_problem(filename):
    nodes = {}
    edges = {}
    origin = None
    destinations = []
    section = None

    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            if line.lower().startswith('nodes:'):
                section = 'nodes'
                continue
            elif line.lower().startswith('edges:'):
                section = 'edges'
                continue
            elif line.lower().startswith('origin:'):
                remainder = line[len('origin:'):].strip()
                if remainder:
                    origin = int(remainder)
                section = 'origin'
                continue
            elif line.lower().startswith('destinations:'):
                remainder = line[len('destinations:'):].strip()
                if remainder:
                    destinations = [int(d.strip()) for d in remainder.split(';') if d.strip()]
                section = 'destinations'
                continue 

            if section == 'nodes':
                parts = line.split(':')
                node_id = int(parts[0].strip())
                coords = parts[1].strip().strip('()')
                x, y = [int(c.strip()) for c in coords.split(',')]
                nodes[node_id] = (x, y)
                edges[node_id] = []

            elif section == 'edges':
                parts = line.split(':')
                node_pair = parts[0].strip().strip('()')
                u, v = [int(n.strip()) for n in node_pair.split(',')]
                cost = int(parts[1].strip())
                if u not in edges:
                    edges[u] = []
                edges[u].append((v, cost))

            elif section == 'origin':
                origin = int(line)

            elif section == 'destinations':
                destinations = [int(d.strip()) for d in line.split(';') if d.strip()]

    return nodes, edges, origin, destinations


def euclidean(nodes, node, destinations):
    """Straight-line distance to the nearest destination. Admissible heuristic."""
    x1, y1 = nodes[node]
    return min(
        math.sqrt((x1 - nodes[d][0]) ** 2 + (y1 - nodes[d][1]) ** 2)
        for d in destinations
    )


def sorted_neighbours(edges, node):
    """Return neighbours sorted by node ID for tie-breaking (ascending)."""
    return sorted(edges.get(node, []), key=lambda x: x[0])


def astar(nodes, edges, origin, destinations):
    destination_set = set(destinations)
    counter = 0
    h0 = euclidean(nodes, origin, destinations)

    # (f_cost, node_id, insertion_counter, node, path, g_cost)
    frontier = [(h0, origin, counter, origin, [origin], 0.0)]
    best_g = {origin: 0.0}
    nodes_created = 1

    while frontier:
        f, _, _, node, path, g = heapq.heappop(frontier)

        # Skip stale entries where a cheaper path was already found
        if g > best_g.get(node, float('inf')):
            continue

        if node in destination_set:
            return node, nodes_created, path

        for neighbour, cost in sorted_neighbours(edges, node):
            new_g = g + cost
            if new_g < best_g.get(neighbour, float('inf')):
                best_g[neighbour] = new_g
                h_n = euclidean(nodes, neighbour, destinations)
                f_n = new_g + h_n
                counter += 1
                heapq.heappush(
                    frontier,
                    (f_n, neighbour, counter, neighbour, path + [neighbour], new_g)
                )
                nodes_created += 1

    return None, nodes_created, []


def main():
    if len(sys.argv) != 3:
        print("Usage: python astar.py <filename> <method>")
        sys.exit(1)

    filename = sys.argv[1]
    method = sys.argv[2].upper()

    if method != 'AS':
        print(f"Unknown method '{method}'. Use AS.")
        sys.exit(1)

    nodes, edges, origin, destinations = parse_problem(filename)
    goal, nodes_created, path = astar(nodes, edges, origin, destinations)

    print(f"{method}")
    if goal is not None:
        print(f"{goal} {nodes_created}")
        print(' -> '.join(str(n) for n in path))
    else:
        print(f"No goal is found.")


if __name__ == '__main__':
    main()
