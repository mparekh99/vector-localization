import numpy as np


class EKFLocalizer:
    def __init__(self):
        self.mu = np.zeros((3,1))

        self.last_pose = None
        