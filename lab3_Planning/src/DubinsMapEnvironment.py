import numpy as np
from matplotlib import pyplot as plt
from Dubins import dubins_path_planning
from Dubins import path_length
from MapEnvironment import MapEnvironment


class DubinsMapEnvironment(MapEnvironment):

    def __init__(self, map_data, curvature=0.05):  # 5
        super(DubinsMapEnvironment, self).__init__(map_data)
        self.curvature = curvature

    def compute_distances(self, start_config, end_configs):
        """
        Compute distance from start_config and end_configs using Dubins path
        @param start_config: tuple of start config
        @param end_configs: list of tuples of end confings
        @return numpy array of distances
        """

        # Implement here
        if isinstance(end_configs, tuple):
            end_configs = [end_configs]
        distance = np.zeros(len(end_configs))
        # print('111111111 ', end_configs, len(end_configs))
        for i in range(len(end_configs)):
            distance[i] = path_length(
                start_config, end_configs[i], self.curvature)
        return distance

    def compute_heuristic(self, config, goal):
        """
        Use the Dubins path length from config to goal as the heuristic distance.
        """

        # Implement here
        distance = path_length(config, goal, self.curvature)
        return distance

    def generate_path(self, config1, config2):
        """
        Generate a dubins path from config1 to config2
        The generated path is not guaranteed to be collision free
        Use dubins_path_planning to get a path
        return: (numpy array of [x, y, yaw], curve length)
        """

        # Implement here
        px, py, pyaw, cost = dubins_path_planning(
            config1, config2, self.curvature)

        path = np.array([px, py, pyaw])
        path = path.T
        return path, cost
