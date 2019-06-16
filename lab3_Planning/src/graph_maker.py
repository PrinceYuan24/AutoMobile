import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import pickle
import os

assert(nx.__version__ == '2.2' or nx.__version__ == '2.1')

def load_graph(filename):
    assert os.path.exists(filename)
    with open(filename, 'rb') as f:
        data = pickle.load(f)
        print('Loaded graph from {}'.format(f))
    return data['G']

def make_graph(env, sampler, connection_radius, num_vertices, directed=True, lazy=False, saveto='graph.pkl'):
    """
    Returns a graph ont he passed environment.
    All vertices in the graph must be collision-free.

    Graph should have node attribute "config" which keeps a configuration in tuple.
    E.g., for adding vertex "0" with configuration np.array([0, 1]),
    G.add_node(0, config=tuple(config))

    To add edges to the graph, call
    G.add_weighted_edges_from(edges)
    where edges is a list of tuples (node_i, node_j, weight),
    where weight is the distance between the two nodes.

    @param env: Map Environment for graph to be made on
    @param sampler: Sampler to sample configurations in the environment
    @param connection_radius: Maximum distance to connect vertices
    @param num_vertices: Minimum number of vertices in the graph.
    @param lazy: If true, edges are made without checking collision.
    @param saveto: File to save graph and the configurations
    """
    G = nx.Graph()
    # Implement here
    # 1. Sample vertices
    samples = sampler.sample(num_vertices)
    print (samples)
    valid = env.state_validity_checker(samples)
    vertices = []
    for i in range(len(valid)):
        if valid[i]:
            vertices.append(tuple(samples[i]))
    for i in range(len(vertices)):
        G.add_node(i, config=vertices[i])

    # diff = len(vertices) - num_vertices
    # while diff != 0:
    #     sample = sampler.sample(1)
    #     vertice = env.state_validity_checker(sample)
    #     if vertice[0] == True:
    #         vertices_list = vertices.tolist()
    #         vertices_list.append(sample[0].tolist())
    #         vertices = np.array(vertices_list)
    #     diff = len(vertices) - num_vertices
    # planning_env.visualize_graph(G)
    # G = nx.Graph()

    # # Implement here
    # # 1. Sample vertices
    # # 2. Connect them with edges
    # samples = sampler.sample(num_vertices)
    # coll_check = env.state_validity_checker(samples).tolist()
    # nocoll_index = indices = [i for i, x in enumerate(coll_check) if x == True]
    # vertices = samples[nocoll_index]
    # diff = len(vertices) - num_vertices
    # while diff != 0:
    #     sample = sampler.sample(1)
    #     vertice = env.state_validity_checker(sample)
    #     if vertice[0] == True:
    #         vertices_list = vertices.tolist()
    #         vertices_list.append(sample[0].tolist())
    #         vertices = np.array(vertices_list)
    #     diff = len(vertices) - num_vertices

    # for i in range(num_vertices):
    #     #print(tuple(vertices[i]))
    #     G.add_node(i, config=tuple(vertices[i]))

    # # 2. Connect them with edges
    edges = []
    for v1 in range(len(vertices)):
        for v2 in range(v1, len(vertices)):
            distance = env.compute_distances(vertices[v1], vertices[v2])
            if lazy:
                if (distance <= connection_radius and distance > 0):
                    edges.append((v1, v2, distance))
                    # G.add_weighted_edges_from([(v1,v2,distance)])
            elif distance <= connection_radius:
                    start = vertices[v1]
                    goal = vertices[v2]
                    vali, dis = env.edge_validity_checker(start, goal)
                    if (vali and dis < connection_radius and dis > 0):
                        edges.append((v1, v2, dis))

    G.add_weighted_edges_from(edges)



    # #test
    # print('--------------------nodes--------------------')
    # print(G.nodes(data=True))
    # print('--------------------edges--------------------')
    # print(G.edges.data)

    # Check for connectivity.
    num_connected_components = len(list(nx.connected_components(G)))
    if not num_connected_components == 1:
        print ("warning, Graph has {} components, not connected".format(num_connected_components))

    # Save the graph to reuse.
    if saveto is not None:
        data = dict(G=G)
        pickle.dump(data, open(saveto, 'wb'))
        print('Saved the graph to {}'.format(saveto))
    return G


def add_node(G, config, env, connection_radius, start_from_config):
    """
    This function should add a node to an existing graph G.
    @param G graph, constructed using make_graph
    @param config Configuration to add to the graph
    @param env Environment on which the graph is constructed
    @param connection_radius Maximum distance to connect vertices
    """
    # new index of the configuration
    index = G.number_of_nodes()
    G.add_node(index, config=tuple(config))
    G_configs = nx.get_node_attributes(G, 'config')
    G_configs = [G_configs[node] for node in G_configs]

    # Implement here
    # Add edges from the newly added node
    config=tuple(config)
    edges = []

    for v in range(index):
        end = G.node[v]['config']
        vali, dis, path= env.edge_validity_checker(config, end)
        if (vali and (dis < connection_radius)):
            edges.append((index, v, dis))
    G.add_weighted_edges_from(edges)

    # Check for connectivity.
    num_connected_components = len(list(nx.connected_components(G)))
    if not num_connected_components == 1:
        print ("warning, Graph has {} components, not connected".format(num_connected_components))

    return G, index
