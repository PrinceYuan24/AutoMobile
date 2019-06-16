import numpy as np

class Sampler:
    def __init__(self, env):
        self.env = env
        self.xlimit = self.env.xlimit
        self.ylimit = self.env.ylimit
        print(self.xlimit)
        print(self.ylimit)

    def sample(self, num_samples):
        """
        Samples configurations.
        Each configuration is (x, y).

        @param num_samples: Number of sample configurations to return
        @return 2D numpy array of size [num_samples x 2]
        """

        # Implement heres
        x_samples = np.random.randint(self.xlimit[0], self.xlimit[-1], size = num_samples)
        y_samples = np.random.randint(self.ylimit[0], self.ylimit[-1], size = num_samples)
        # x_samples = np.random.randint(1700, 2200, size = num_samples)
        # y_samples = np.random.randint(2300, 2800, size = num_samples)
        samples = np.array([x_samples, y_samples])
        samples = samples.T
        print(samples)
        return samples
