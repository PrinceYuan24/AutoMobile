# This is modified from networkx implementation
# https://networkx.github.io/documentation/stable/_modules/networkx/algorithms/shortest_paths/astar.html#astar_path
from heapq import heappush, heappop
from itertools import count
import numpy as np
import networkx as nx

def astar_path(G, MapEnvironment, source, target, weight, heuristic=None):
    """Return a list of nodes in a shortest path between source and target
    using the A* ("A-star") algorithm.

    There may be more than one shortest path.  This returns only one.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node for path

    target : node
       Ending node for path

    weight: function. (validity, weight) of an edge is
       the value returned by the function. The function must
       accept exactly two positional arguments:
       the two endpoints of an edge.
       The function must return a (boolean, number).

    heuristic : function
       A function to evaluate the estimate of the distance
       from the a node to the target.  The function takes
       two nodes arguments and must return a number.

    Raises
    ------
    NetworkXNoPath
        If no path exists between source and target.
    """

    if source not in G or target not in G:
        msg = 'Either source {} or target {} is not in G'
        raise nx.NodeNotFound(msg.format(source, target))

    if heuristic is None:
        # The default heuristic is h=0 - same as Dijkstra's algorithm
        def heuristic(u, v):
            return 0

    push = heappush
    pop = heappop

    # The queue stores priority, node, cost to reach, and parent.
    # Uses Python heapq to keep in priority order.
    # Add a counter to the queue to prevent the underlying heap from
    # attempting to compare the nodes themselves. The hash breaks ties in the
    # priority and is guaranteed unique for all nodes in the graph.
    c = count()
    queue = [(0, next(c), source, 0, None)]

    # Maps enqueued nodes to distance of discovered paths and the
    # computed heuristics to target. We avoid computing the heuristics
    # more than once and inserting the node into the queue too many times.
    enqueued = {}
    # Maps explored nodes to parent closest to the source.
    explored = {}
    edge_count=0
    while queue:
        # Pop the smallest item from queue.
        _, __, curnode, dist, parent = pop(queue)

        # Implement here using astar.py as your reference.
        if curnode == target:
            path = [curnode]
            node = parent
            while node is not None:
                path.append(node)
                node = explored[node]
            path.reverse()
            print('edges', edge_count)
            return path

        if curnode in explored:
            continue

        if parent != None:
            valid, dis, path= MapEnvironment.edge_validity_checker(G.node[curnode]['config'], G.node[parent]['config'])
            if not valid:
                continue

        explored[curnode] = parent

        for neighbor, w in G[curnode].items():
            if neighbor in explored:
                continue
            edge_count+=1
            ncost = dist + w.get('weight', 5)
            if neighbor in enqueued:
                qcost, h = enqueued[neighbor]
                # if qcost <= ncost, a less costly path from the
                # neighbor to the source was already determined.
                # Therefore, we won't attempt to push this neighbor
                # to the queue
                if qcost <= ncost:
                    continue
            else:
                h = heuristic(neighbor, target)

            enqueued[neighbor] = ncost, h
            push(queue, (ncost + h, next(c), neighbor, ncost, curnode))

    #raise nx.NetworkXNoPath("Node %s not reachable from %s" % (source, target))
    return False

def path_length(G, path):
    """Return the length of the path

    Parameters
    ----------
    G : NetworkX graph

    path : a path to be evaluated.
    """
    return sum(G[u][v].get('weight', 5) for u, v in zip(path[:-1], path[1:]))

# #wzy
# def collision_check(config1, config2):
#     """
#     Checks whether the path between config 1 and config 2 is valid
#     """
#     path = generate_path(config1, config2)

#     if not np.all(state_validity_checker(path)):
#         return False
#     return True

# #wzy
# def generate_path(config1, config2):
#     stepsize = 0.5
#     config1 = np.array(config1)
#     config2 = np.array(config2)
#     dist = np.linalg.norm(config2 - config1)

#     direction = (config2 - config1) / dist
#     steps = dist // stepsize + 1

#     waypoints = np.array([np.linspace(config1[i], config2[i], steps) for i in range(2)]).transpose()

#     return waypoints

# def state_validity_checker(configs):
#     """
#     @param configs: 2D array of [num_configs, dimension].
#     Each row contains a configuration
#     @return numpy list of boolean values. True for valid states.
#     """
#     if len(configs.shape) == 1:
#         configs = configs.reshape(1, -1)

#     # Implement here
#     # 1. Check for state bounds within xlimit and ylimit

#     # 2. Check collision
#     validity = np.ones(len(configs))
#     # state_check
#     state_invalid_index = [i for i, e in enumerate(configs)\
#     if (e[0] > self.xlimit[1] or e[0] < self.xlimit[0]\
#     or e[1] > self.ylimit[1] or e[1] < self.ylimit[0])]
#     validity[state_invalid_index] = False
#     # collission check
#     collission_invalid_index = [i for i, e in enumerate(configs)\
#     if (self.map[int(e[0]+0.5), int(e[1]+0.5)] == True)]
#     validity[collission_invalid_index] = False

#     validity = np.asarray(validity)
#     return validity
