"""A* pathfinding on a 2D grid."""

import heapq
from typing import List, Tuple, Set, Optional
from core.waypoint import Waypoint


class PathPlanner:
    """Plans collision-free paths using A* algorithm."""
    
    def __init__(self, grid_size: Tuple[int, int], obstacles: List[Tuple[int, int]]):
        self.grid_size = grid_size
        self.obstacles = set(obstacles)
    
    def plan(self, start: Waypoint, goal: Waypoint) -> List[Waypoint]:
        """A* pathfinding from start to goal."""
        start_cell = self._to_grid(start)
        goal_cell = self._to_grid(goal)
        
        if start_cell == goal_cell:
            return [start, goal]
        
        if goal_cell in self.obstacles:
            return [start, goal]
        
        path_cells = self._astar(start_cell, goal_cell)
        
        if not path_cells:
            return [start, goal]
        
        path_waypoints = [self._to_waypoint(cell) for cell in path_cells]
        return path_waypoints
    
    def _astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Core A* implementation."""
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        closed_set: Set[Tuple[int, int]] = set()
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            if current == goal:
                return self._reconstruct_path(came_from, current)
            
            closed_set.add(current)
            
            for neighbor in self._get_neighbors(current):
                if neighbor in closed_set or neighbor in self.obstacles:
                    continue
                
                tentative_g = g_score[current] + self._cost(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return []
    
    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic."""
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        return (dx * dx + dy * dy) ** 0.5
    
    def _cost(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Movement cost between adjacent cells."""
        dx = abs(b[0] - a[0])
        dy = abs(b[1] - a[1])
        return 1.414 if (dx + dy == 2) else 1.0
    
    def _get_neighbors(self, cell: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get 8-connected neighbors."""
        x, y = cell
        neighbors = []
        
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                nx, ny = x + dx, y + dy
                
                if 0 <= nx < self.grid_size[0] and 0 <= ny < self.grid_size[1]:
                    neighbors.append((nx, ny))
        
        return neighbors
    
    def _reconstruct_path(self, came_from: dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct path from A* search."""
        path = [current]
        
        while current in came_from:
            current = came_from[current]
            path.append(current)
        
        path.reverse()
        return path
    
    def _to_grid(self, waypoint: Waypoint) -> Tuple[int, int]:
        """Convert continuous waypoint to grid cell."""
        return (int(waypoint.x), int(waypoint.y))
    
    def _to_waypoint(self, cell: Tuple[int, int]) -> Waypoint:
        """Convert grid cell to waypoint at cell center."""
        return Waypoint(x=float(cell[0]) + 0.5, y=float(cell[1]) + 0.5)
