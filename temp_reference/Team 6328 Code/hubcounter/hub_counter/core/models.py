"""Shared data models."""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class BallCounts:
    """Ball count state across all channels."""

    channels: list[int] = field(default_factory=lambda: [0, 0, 0, 0])
    total: int = 0
    paused_total: int = 0

    def __post_init__(self):
        """Ensure channels list has 4 elements."""
        if len(self.channels) != 4:
            self.channels = [0, 0, 0, 0]
        self.total = sum(self.channels)

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {"channels": self.channels.copy(), "total": self.total, "paused_total": self.paused_total}

    @classmethod
    def from_dict(cls, data: dict) -> "BallCounts":
        """Create from dictionary."""
        return cls(
            channels=data.get("channels", [0, 0, 0, 0]),
            paused_total=data.get("paused_total", 0),
        )


@dataclass
class MatchInfo:
    """FRC match information from NetworkTables."""

    match_time: float = 0.0
    is_autonomous: bool = False
    is_teleop: bool = False
    is_disabled: bool = True
    match_number: int = 0
    event_name: str = ""

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "match_time": self.match_time,
            "is_autonomous": self.is_autonomous,
            "is_teleop": self.is_teleop,
            "is_disabled": self.is_disabled,
            "match_number": self.match_number,
            "event_name": self.event_name,
        }


@dataclass
class SystemStatus:
    """Overall system status."""

    counts: BallCounts = field(default_factory=BallCounts)
    nt_connected: bool = False
    match_info: Optional[MatchInfo] = None
    led_active: bool = False

    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "counts": self.counts.to_dict(),
            "nt_connected": self.nt_connected,
            "match_info": self.match_info.to_dict() if self.match_info else None,
            "led_active": self.led_active,
        }
