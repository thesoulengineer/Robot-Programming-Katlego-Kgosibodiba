"""Waypoint data structures and management."""

from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class Waypoint:
    """Target position in 2D space."""
    x: float
    y: float


class WaypointQueue:
    """Manages sequential waypoint navigation."""
    
    def __init__(self, waypoints: List[Waypoint]):
        self.waypoints = waypoints
        self.current_index = 0
    
    def get_next_segment(self) -> Tuple[Optional[Waypoint], Optional[Waypoint]]:
        """Get current start and goal waypoint pair."""
        if self.current_index >= len(self.waypoints) - 1:
            return None, None
        
        start = self.waypoints[self.current_index]
        goal = self.waypoints[self.current_index + 1]
        return start, goal
    
    def advance(self) -> None:
        """Move to next waypoint segment."""
        self.current_index += 1
    
    def is_complete(self) -> bool:
        """Check if all waypoints have been visited."""
        return self.current_index >= len(self.waypoints) - 1
