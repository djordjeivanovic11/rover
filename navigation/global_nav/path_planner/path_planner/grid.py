import numpy as np
from typing import Tuple, List
from .utils import get_movements_4n, get_movements_8n

OBSTACLE = 255
UNOCCUPIED = 0

class OccupancyGridMap:
    """
    Simple 2D occupancy grid wrapper for D* Lite planning.
    """
    def __init__(self,
                 width: int,
                 height: int,
                 connectivity: str = '8N'):
        """
        :param width:  number of cells in x (columns)
        :param height: number of cells in y (rows)
        :param connectivity: '4N' or '8N'
        """
        self.width  = width
        self.height = height
        self.connectivity = connectivity
        self.occupancy = np.zeros((height, width), dtype=np.uint8)

    def set_from_ros(self,
                     data: List[int],
                     obstacle_thresh: int):
        """
        Load data[] from nav_msgs/OccupancyGrid into our map.
        :param data: flat list of cell values (row-major)
        :param obstacle_thresh: values >= this are treated as obstacles
        """
        grid = np.array(data, dtype=int).reshape((self.height, self.width))
        self.occupancy[:, :] = UNOCCUPIED
        self.occupancy[grid >= obstacle_thresh] = OBSTACLE

    def in_bounds(self, cell: Tuple[int,int]) -> bool:
        """
        True if the given (row,col) is inside the grid.
        """
        r, c = cell
        return 0 <= r < self.height and 0 <= c < self.width

    def is_unoccupied(self, cell: Tuple[int,int]) -> bool:
        """
        True if the cell is free to traverse.
        """
        r, c = cell
        return self.occupancy[r, c] == UNOCCUPIED

    def succ(self,
             cell: Tuple[int,int],
             avoid_obstacles: bool = False) -> List[Tuple[int,int]]:
        """
        Get all valid successors of 'cell'.
        :param cell: (row, col) of current cell
        :param avoid_obstacles: if True, only return free cells
        :return: list of (row, col) neighbours
        """
        r, c = cell
        moves = (get_movements_4n if self.connectivity == '4N'
                 else get_movements_8n)(r, c)

        valid = []
        for nr, nc, _cost in moves:
            if not self.in_bounds((nr, nc)):
                continue
            if avoid_obstacles and not self.is_unoccupied((nr,nc)):
                continue
            valid.append((nr, nc))
        return valid
