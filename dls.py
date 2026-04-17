import sys
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

def depth_limited_search(origin, destinations, edges, limit):
    CUTOFF = object()   # sentinel — unambiguously "stopped by depth limit"

    nodes_created = 0   # count every node pushed onto the stack

    def recursive_dls(node_id, path, visited, depth):
        nonlocal nodes_created

        # ---- goal check ----
        if node_id in destinations:
            return (node_id, path)

        # ---- depth limit reached ----
        if depth == 0:
            return CUTOFF

        # ---- expand children ----
        # Sort neighbours ascending by node ID to satisfy the tie-breaking rule.
        neighbours = sorted(edges.get(node_id, []), key=lambda x: x[0])

        cutoff_occurred = False

        for (neighbour, _cost) in neighbours:
            if neighbour in visited:
                continue                        # skip already-visited nodes

            nodes_created += 1
            visited.add(neighbour)

            result = recursive_dls(neighbour, path + [neighbour], visited, depth - 1)

            if result is CUTOFF:
                cutoff_occurred = True
            elif result is not None:
                return result                   # propagate success upwards

            visited.discard(neighbour)

        return CUTOFF if cutoff_occurred else None

    # ---- kick off the search ----
    initial_path = [origin]
    nodes_created += 1      # the root node counts
    result = recursive_dls(origin, initial_path, {origin}, limit)

    if result is CUTOFF or result is None:
        return (None, [], nodes_created)

    goal, path = result
    return (goal, path, nodes_created)

def main():
    if len(sys.argv) != 3:
        print("Usage: python search_cus1_dls.py <filename> <method>")
        sys.exit(1)

    filename = sys.argv[1]
    method = sys.argv[2].upper()

    if method != "CUS1":
        print(f"Unknown method '{method}'. Use CUS1.")
        sys.exit(1)

    nodes, edges, origin, destinations = parse_problem(filename)
    goal, path, nodes_created = depth_limited_search(origin, destinations, edges, limit=50)

    print(f"{method}")
    if goal is not None:
        print(f"{goal} {nodes_created}")
        print(" -> ".join(str(n) for n in path))
    else:
        print("No goal is found.")

if __name__ == "__main__":
    main()