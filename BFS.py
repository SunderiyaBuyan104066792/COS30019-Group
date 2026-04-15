import sys
from collections import deque

from utils import *



# ______________________________________________________________________________


class Node:
    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state. Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node. Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        """[Figure 3.10]"""
        next_state = problem.result(self.state, action)
        next_node = Node(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        return next_node

    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_graph_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        # We use the hash value of the state
        # stored in the node instead of the node
        # object itself to quickly search a node
        # with the same state in a Hash Table
        return hash(self.state)

def Breadth_first_search(nodes, edges, origin, destination):
    nodes_created = 1
    root = Node(origin, None,0,0,0)
    if root.state in destination:
        return root, nodes_created
    Frontier = deque([root])   
    visited = set()
    while Frontier:
        current = Frontier.popleft()
        if current.state in destination:
            return current, nodes_created
        
        if current.state not in visited:
            visited.add(current.state)

            for next_state in edges.get(current.state, []):
                if next_state not in visited and next_state not in [node.state for node in Frontier]:
                    child = Node(next_state, current, 0, current.depth + 1, nodes_created)
                    Frontier.append(child)
                    nodes_created += 1

    return None, nodes_created


def Main():
    file = sys.argv[1]
    method = sys.argv[2]
    if method == "BFS":
        solution, path, goal = Breadth_first_search()
        print(f"{file}\n {goal} {solution} \n {path}")

      
    