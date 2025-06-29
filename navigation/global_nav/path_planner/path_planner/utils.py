import math
from typing import List, Tuple

def heuristic(p: Tuple[int, int], q: Tuple[int, int]) -> float:
    """
    Euclidean distance between two grid cells.
    :param p: (row, col) of first cell
    :param q: (row, col) of second cell
    :return: straight-line distance in cell units
    """
    return math.hypot(p[0] - q[0], p[1] - q[1])


def get_movements_4n(x: int, y: int) -> List[Tuple[int, int, float]]:
    """
    4-connectivity: up/down/left/right with unit cost.
    :return: List of (new_x, new_y, movement_cost)
    """
    return [
        (x + 1, y,     1.0),
        (x - 1, y,     1.0),
        (x,     y + 1, 1.0),
        (x,     y - 1, 1.0),
    ]


def get_movements_8n(x: int, y: int) -> List[Tuple[int, int, float]]:
    """
    8-connectivity: includes diagonals.
    Straight moves cost 1.0, diagonals cost sqrt(2).
    :return: List of (new_x, new_y, movement_cost)
    """
    diag = math.sqrt(2)
    return [
        (x + 1, y,     1.0),
        (x - 1, y,     1.0),
        (x,     y + 1, 1.0),
        (x,     y - 1, 1.0),
        (x + 1, y + 1, diag),
        (x - 1, y + 1, diag),
        (x - 1, y - 1, diag),
        (x + 1, y - 1, diag),
    ]
