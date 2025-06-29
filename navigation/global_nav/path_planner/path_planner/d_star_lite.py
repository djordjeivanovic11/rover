import numpy as np
from typing import Tuple, List
from .priority_queue import PriorityQueue, Priority
from .grid import OccupancyGridMap
from .utils import heuristic

class DStarLite:
    """
    Incremental D* Lite planner on a 2D OccupancyGridMap.
    """

    def __init__(self,
                 grid: OccupancyGridMap,
                 start: Tuple[int,int],
                 goal:  Tuple[int,int]):
        """
        :param grid:  an OccupancyGridMap instance
        :param start: (row, col) start cell
        :param goal:  (row, col) goal cell
        """
        self.grid   = grid
        self.start  = start
        self.goal   = goal
        self.k_m    = 0.0

        h, w = grid.height, grid.width
        # g and rhs values
        self.rhs = np.full((h, w), np.inf)
        self.g   = np.full((h, w), np.inf)

        # goal has zero rhs
        self.rhs[self.goal] = 0.0

        # priority queue of vertices to process
        self.U = PriorityQueue()
        self.U.insert(self.goal, self._calculate_key(self.goal))

    def _calculate_key(self, s: Tuple[int,int]) -> Priority:
        """
        Compute lexicographic key for a cell s = (r,c).
        k1 = min(g[s], rhs[s]) + h(start,s) + k_m
        k2 = min(g[s], rhs[s])
        """
        g_rhs = min(self.g[s], self.rhs[s])
        return Priority(g_rhs + heuristic(self.start, s) + self.k_m, g_rhs)

    def _cost(self, u: Tuple[int,int], v: Tuple[int,int]) -> float:
        """
        Cost of moving from u to v: infinite if either cell blocked,
        else Euclidean distance.
        """
        if not self.grid.is_unoccupied(u) or not self.grid.is_unoccupied(v):
            return np.inf
        return heuristic(u, v)

    def _update_vertex(self, u: Tuple[int,int]):
        """
        Update or remove u in the priority queue based on g[u] vs rhs[u].
        """
        if u != self.goal:
            # recompute rhs[u] from successors
            succ = self.grid.succ(u)
            self.rhs[u] = min(self._cost(u, s) + self.g[s] for s in succ)

        # remove from queue if present
        self.U.remove(u)
        # reinsert if key needs tracking
        if self.g[u] != self.rhs[u]:
            self.U.insert(u, self._calculate_key(u))

    def compute_shortest_path(self):
        """
        Core loop: pop vertices until start is consistent and
        queue top key ≥ key(start).
        """
        while (self.U.top_key() < self._calculate_key(self.start)
               or self.rhs[self.start] != self.g[self.start]):
            u = self.U.pop()
            if u is None:
                break

            k_old = self._calculate_key(u)
            k_new = self._calculate_key(u)
            if k_old < k_new:
                # key increased
                self.U.insert(u, k_new)
            elif self.g[u] > self.rhs[u]:
                # overconsistent: set g = rhs
                self.g[u] = self.rhs[u]
                for p in self.grid.succ(u):
                    self._update_vertex(p)
            else:
                # underconsistent: set g = ∞ and update predecessors + u
                g_old = self.g[u]
                self.g[u] = np.inf
                preds = self.grid.succ(u) + [u]
                for p in preds:
                    if self.rhs[p] == self._cost(p, u) + g_old:
                        self._update_vertex(p)
                self._update_vertex(u)

    def get_path(self) -> List[Tuple[int,int]]:
        """
        Extract a path from start to goal following the min-successor rule.
        Must call compute_shortest_path() first.
        """
        path = []
        s = self.start
        if self.rhs[s] == np.inf:
            return []  # no path

        while s != self.goal:
            path.append(s)
            succ = self.grid.succ(s)
            # pick successor minimizing cost(s,succ)+g[succ]
            s = min(succ, key=lambda x: self._cost(s, x) + self.g[x])
        path.append(self.goal)
        return path
