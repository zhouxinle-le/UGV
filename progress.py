"""Project positions onto consecutive segments of a single lap."""

import numpy as np


STOP_DISTANCE = 0.5  # m of route remaining when the vehicle applies the brake
APPROACH_DECEL = 0.5  # m/s² used for the stopping speed profile


class RouteProgress:
    def __init__(self, points):
        self.points = np.asarray(points)
        self.vectors = np.diff(self.points, axis=0)
        self.lengths = np.linalg.norm(self.vectors, axis=1)
        self.stations = np.r_[0.0, self.lengths.cumsum()]
        self.station = 0.0

    def update(self, x, y):
        # Search locally in lap order so the shared start/finish cannot skip a lap.
        first = max(0, self.stations.searchsorted(self.station - 10.0) - 1)
        last = self.stations.searchsorted(self.station + 15.0)
        segments = slice(first, last)
        offsets = np.array([x, y]) - self.points[:-1][segments]
        vectors = self.vectors[segments]
        fraction = np.clip(np.sum(offsets * vectors, axis=1) / self.lengths[segments]**2, 0, 1)
        squared = np.sum((offsets - fraction[:, None] * vectors)**2, axis=1)
        best = squared.argmin()
        index = first + best
        ratio = fraction[best]
        self.station = max(self.station, self.stations[index] + ratio * self.lengths[index])
        return float(self.station), float(np.sqrt(squared[best])), int(index), float(ratio)
