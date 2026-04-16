
def depth_first_search(nodes, edges, origin, destinations):
    nodes_created = 1
    root = Node(origin, None, 0, 0)

    if root.state in destinations:
        return root, nodes_created, [root.state]

    frontier = [root]
    visited = set()

    while frontier:
        current = frontier.pop()

        if current.state in visited:
            continue
        visited.add(current.state)

        if current.state in destinations:
            return current, nodes_created

        for next_state, edge_cost in sorted(edges.get(current.state, []), key=lambda item: item[0], reverse=True):
            if next_state not in visited:
                child = Node(next_state, current, 0, current.depth + 1, nodes_created)
                nodes_created += 1
                frontier.append(child)

    return None, nodes_created, []

# Number of Tests: 2

def dfs_tests():
    print("=" * 40)
    print("RUNNING DFS TESTS....")
    print("=" * 40)


    # TEST 1:
    print("\nTest1: No path exists")
    test_graph_1 = {
        1: [(2, 3)]
        # no possible path
    }
    test_nodes_1 = {1: (1,1), 2: (5,5), 3: (9,9)}
    test_origin_1 = 1
    test_destinations_1 = {3}

    result, nodes_created, path = depth_first_search(
        test_nodes_1, test_graph_1, test_origin_1, test_destinations_1
    )

    if result is None:
        print("Pass - no solution found")

    else:
        print("Fail - no solution is expected")
    print(f"Nodes created: {nodes_created}")



    # TEST
    print("\nTest 2: Origin is the destination")
    test_graph_2 = {
        1: [(2,4)]
    }
    test_nodes_2 = {1: (1,1), 2: (5,5)}
    test_origin_2 = 1
    test_destinations_2 = {1}

    result, nodes_created, path = depth_first_search(
        test_nodes_2, test_graph_2, test_origin_2, test_destinations_2
    )

    if result is not None and result.state == 1:
        print("Pass - origin correctly idenitifed")
        print(f"Path: {build_path(result)}")
    else:
        print("Fail - should return origin as goal")
    print(f"Nodes created: {nodes_created}")

    print("\n" + "=" * 40)
    print("Tests for DFS complete.")
    print("=" * 40)







# for main:
# if method == "DFS":
#     result_node, nodes_created, path = depth_first_search(nodes, edges, origin, destinations)


dfs_tests()
