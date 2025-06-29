import numpy as np
import math

def radius_outlier_filter(points, radius=0.25, min_neighbors=2):
    """
    Simple PCL-style radius filter in NumPy.
    points: (N,3) array in metres (x,y,z)
    """
    if len(points) < min_neighbors:
        return points
    # build k-d tree in a naive way (N^2) because N per ROI is small (few k)
    keep_mask = np.zeros(len(points), dtype=bool)
    for i, p in enumerate(points):
        d2 = np.sum((points - p)**2, axis=1)
        if np.count_nonzero(d2 < radius*radius) >= min_neighbors:
            keep_mask[i] = True
    return points[keep_mask]
