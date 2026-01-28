"""
Waypoint Manager

Manages navigation goals and multi-waypoint missions.

Handles:
- GPS coordinate to local coordinate conversion
- Multi-stop routes (A → B → C → ...)
- Waypoint reached detection
- Mission status tracking
"""

import math
import time
from typing import List, Tuple, Optional
from dataclasses import dataclass, field
from enum import Enum

try:
    from perception.transforms import GPSConverter
except ImportError:
    GPSConverter = None


class WaypointStatus(Enum):
    PENDING = 0
    ACTIVE = 1
    REACHED = 2
    SKIPPED = 3
    FAILED = 4


class MissionStatus(Enum):
    IDLE = 0
    RUNNING = 1
    PAUSED = 2
    COMPLETED = 3
    FAILED = 4


@dataclass
class Waypoint:
    """A navigation waypoint."""
    x: float                            # meters (local frame)
    y: float
    name: str = ""
    tolerance: float = 0.5              # meters - how close counts as "reached"
    stop_duration: float = 0.0          # seconds to wait at waypoint
    max_speed: float = 1.0             # speed limit approaching this waypoint
    status: WaypointStatus = WaypointStatus.PENDING
    reached_time: float = 0.0

    @property
    def position(self) -> Tuple[float, float]:
        return (self.x, self.y)

    def distance_to(self, x: float, y: float) -> float:
        return math.sqrt((self.x - x)**2 + (self.y - y)**2)


@dataclass
class Mission:
    """A navigation mission with multiple waypoints."""
    name: str
    waypoints: List[Waypoint] = field(default_factory=list)
    loop: bool = False                  # Repeat when done
    status: MissionStatus = MissionStatus.IDLE
    current_index: int = 0
    start_time: float = 0.0
    end_time: float = 0.0

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        if 0 <= self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None

    @property
    def progress(self) -> float:
        """Progress 0.0 to 1.0."""
        if not self.waypoints:
            return 1.0
        reached = sum(1 for w in self.waypoints if w.status == WaypointStatus.REACHED)
        return reached / len(self.waypoints)


class WaypointManager:
    """
    Manages waypoints and missions.

    Usage:
        manager = WaypointManager()

        # Set GPS origin
        manager.set_gps_origin(48.8566, 2.3522)

        # Add waypoints by local coordinates
        manager.add_waypoint(5.0, 3.0, name="Kitchen")
        manager.add_waypoint(10.0, 0.0, name="Room 101")

        # Or add by GPS
        manager.add_waypoint_gps(48.8567, 2.3523, name="Lobby")

        # Start mission
        manager.start()

        # In control loop:
        goal = manager.get_current_goal()
        if goal is None:
            print("Mission complete!")

        # When reached a waypoint:
        manager.update_position(robot_x, robot_y)
    """

    def __init__(self):
        self._mission: Optional[Mission] = None
        self._gps_converter: Optional[object] = None
        self._waiting_until: float = 0.0

    def set_gps_origin(self, lat: float, lon: float, alt: float = 0.0):
        """Set GPS origin for coordinate conversion."""
        if GPSConverter:
            self._gps_converter = GPSConverter(lat, lon, alt)

    def create_mission(self, name: str = "Mission", loop: bool = False):
        """Create a new mission."""
        self._mission = Mission(name=name, loop=loop)

    def add_waypoint(self, x: float, y: float, name: str = "",
                     tolerance: float = 0.5, stop_duration: float = 0.0,
                     max_speed: float = 1.0):
        """Add waypoint in local coordinates (meters)."""
        if self._mission is None:
            self.create_mission()

        wp = Waypoint(
            x=x, y=y, name=name or f"WP{len(self._mission.waypoints)+1}",
            tolerance=tolerance, stop_duration=stop_duration,
            max_speed=max_speed
        )
        self._mission.waypoints.append(wp)

    def add_waypoint_gps(self, lat: float, lon: float, name: str = "",
                         tolerance: float = 0.5, stop_duration: float = 0.0):
        """Add waypoint by GPS coordinates."""
        if self._gps_converter is None:
            raise RuntimeError("GPS origin not set. Call set_gps_origin() first.")

        east, north, _ = self._gps_converter.gps_to_local(lat, lon)
        self.add_waypoint(east, north, name=name, tolerance=tolerance,
                         stop_duration=stop_duration)

    def start(self):
        """Start the mission."""
        if self._mission is None or not self._mission.waypoints:
            return

        self._mission.status = MissionStatus.RUNNING
        self._mission.current_index = 0
        self._mission.start_time = time.time()

        # Set first waypoint as active
        self._mission.waypoints[0].status = WaypointStatus.ACTIVE

    def pause(self):
        """Pause the mission."""
        if self._mission:
            self._mission.status = MissionStatus.PAUSED

    def resume(self):
        """Resume the mission."""
        if self._mission and self._mission.status == MissionStatus.PAUSED:
            self._mission.status = MissionStatus.RUNNING

    def stop(self):
        """Stop and reset the mission."""
        if self._mission:
            self._mission.status = MissionStatus.IDLE
            self._mission.current_index = 0
            for wp in self._mission.waypoints:
                wp.status = WaypointStatus.PENDING

    def update_position(self, x: float, y: float) -> bool:
        """
        Update robot position and check waypoint progress.

        Args:
            x, y: Current robot position

        Returns:
            True if a waypoint was just reached
        """
        if self._mission is None or self._mission.status != MissionStatus.RUNNING:
            return False

        current_wp = self._mission.current_waypoint
        if current_wp is None:
            return False

        # Check if waiting at a waypoint
        if self._waiting_until > 0:
            if time.time() >= self._waiting_until:
                self._waiting_until = 0.0
                return self._advance_waypoint()
            return False

        # Check if reached current waypoint
        dist = current_wp.distance_to(x, y)
        if dist <= current_wp.tolerance:
            current_wp.status = WaypointStatus.REACHED
            current_wp.reached_time = time.time()

            # Wait if needed
            if current_wp.stop_duration > 0:
                self._waiting_until = time.time() + current_wp.stop_duration
                return True

            return self._advance_waypoint()

        return False

    def _advance_waypoint(self) -> bool:
        """Move to next waypoint."""
        self._mission.current_index += 1

        if self._mission.current_index >= len(self._mission.waypoints):
            if self._mission.loop:
                # Reset and loop
                self._mission.current_index = 0
                for wp in self._mission.waypoints:
                    wp.status = WaypointStatus.PENDING
                self._mission.waypoints[0].status = WaypointStatus.ACTIVE
            else:
                self._mission.status = MissionStatus.COMPLETED
                self._mission.end_time = time.time()
            return True

        # Activate next waypoint
        self._mission.waypoints[self._mission.current_index].status = WaypointStatus.ACTIVE
        return True

    def get_current_goal(self) -> Optional[Tuple[float, float]]:
        """Get current goal position, or None if mission complete."""
        if self._mission is None or self._mission.status != MissionStatus.RUNNING:
            return None

        wp = self._mission.current_waypoint
        return wp.position if wp else None

    def get_current_max_speed(self) -> float:
        """Get speed limit for current waypoint."""
        if self._mission and self._mission.current_waypoint:
            return self._mission.current_waypoint.max_speed
        return 1.0

    def get_mission_status(self) -> dict:
        """Get mission status summary."""
        if self._mission is None:
            return {"status": "NO_MISSION"}

        m = self._mission
        return {
            "status": m.status.name,
            "name": m.name,
            "progress": f"{m.progress*100:.0f}%",
            "current_waypoint": m.current_waypoint.name if m.current_waypoint else "N/A",
            "waypoints_total": len(m.waypoints),
            "waypoints_reached": sum(1 for w in m.waypoints if w.status == WaypointStatus.REACHED),
            "elapsed": f"{time.time() - m.start_time:.1f}s" if m.start_time > 0 else "N/A"
        }

    def get_all_waypoints(self) -> List[Waypoint]:
        """Get all waypoints."""
        return self._mission.waypoints if self._mission else []

    @property
    def is_mission_active(self) -> bool:
        return self._mission is not None and self._mission.status == MissionStatus.RUNNING

    @property
    def is_mission_complete(self) -> bool:
        return self._mission is not None and self._mission.status == MissionStatus.COMPLETED

    @property
    def is_waiting(self) -> bool:
        return self._waiting_until > 0 and time.time() < self._waiting_until