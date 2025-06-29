import heapq
from typing import Any

class Priority:
    """Lexicographic key (k1, k2) for D* Lite."""
    def __init__(self, k1: float, k2: float):
        self.k1, self.k2 = k1, k2
    def __lt__(self, other: "Priority") -> bool:
        return (self.k1, self.k2) < (other.k1, other.k2)

class _Entry:
    __slots__ = ("priority","vertex","removed")
    def __init__(self, priority: Priority, vertex: Any):
        self.priority = priority
        self.vertex   = vertex
        self.removed  = False

    def __lt__(self, other: "_Entry") -> bool:
        # heapq uses this to sort entries
        return self.priority < other.priority

class PriorityQueue:
    """A min-heap priority queue with lazy removal."""
    def __init__(self):
        self._heap = []              # list of _Entry
        self._entry_finder = {}      # vertex → _Entry

    def insert(self, vertex: Any, priority: Priority):
        """Add or update the priority of `vertex`."""
        if vertex in self._entry_finder:
            # mark the old entry as removed
            self._entry_finder[vertex].removed = True
        entry = _Entry(priority, vertex)
        self._entry_finder[vertex] = entry
        heapq.heappush(self._heap, entry)

    def remove(self, vertex: Any):
        """Logically remove `vertex` if present."""
        entry = self._entry_finder.pop(vertex, None)
        if entry:
            entry.removed = True

    def top(self) -> Any:
        """Return the vertex with smallest key without removing it."""
        while self._heap and self._heap[0].removed:
            heapq.heappop(self._heap)
        return self._heap[0].vertex if self._heap else None

    def top_key(self) -> Priority:
        """Return the smallest key (k1,k2), or (inf,inf) if empty."""
        while self._heap and self._heap[0].removed:
            heapq.heappop(self._heap)
        if not self._heap:
            return Priority(float("inf"), float("inf"))
        return self._heap[0].priority

    def pop(self) -> Any:
        """Remove and return the vertex with smallest key, or None."""
        while self._heap:
            entry = heapq.heappop(self._heap)
            if not entry.removed:
                self._entry_finder.pop(entry.vertex, None)
                return entry.vertex
        return None
