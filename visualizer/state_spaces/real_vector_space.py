import numpy as np
from state_spaces.state_space import StateSpace
from math import pi


class RealVectorSpace(StateSpace):
    def __init__(self, dimensions) -> None:
        self.dimensions = dimensions
        self.range = [-pi, pi]

    def get_dimensions(self) -> int:
        return self.dimensions

    def get_qrand(self):
        return np.random.uniform(self.range[0], self.range[1],
                                 size=self.dimensions)

    def get_qnew(self, qnear, qrand, eps):
        a = qnear
        b = qrand
        return a + eps*(b-a)/np.linalg.norm(b-a)

    def equal(self, state1, state2, eps):
        return self.distance(state1, state2) < eps

    def distance(self, s1, s2):
        return np.linalg.norm(s2-s1)
