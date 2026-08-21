"""Thread-safe ball counter."""

import logging
import threading
from typing import Callable, Optional

from ..core.models import BallCounts

logger = logging.getLogger(__name__)

# Type alias for count change callback
CountChangeCallback = Callable[[BallCounts], None]


class BallCounter:
    """Thread-safe ball count manager."""

    def __init__(self, num_channels: int = 4):
        """Initialize the ball counter.

        Args:
            num_channels: Number of channels to track (default 4).
        """
        self._num_channels = num_channels
        self._counts = [0] * num_channels
        self._paused_total = 0
        self._lock = threading.Lock()
        self._callbacks: list[CountChangeCallback] = []

    @property
    def counts(self) -> BallCounts:
        """Get current ball counts (thread-safe)."""
        with self._lock:
            return BallCounts(channels=self._counts.copy(), paused_total=self._paused_total)

    def increment(self, channel: int) -> BallCounts:
        """Increment count for a specific channel.

        Args:
            channel: Channel index (0-based).

        Returns:
            Updated BallCounts.

        Raises:
            ValueError: If channel is out of range.
        """
        if not 0 <= channel < self._num_channels:
            raise ValueError(f"Channel {channel} out of range (0-{self._num_channels - 1})")

        with self._lock:
            self._counts[channel] += 1
            counts = BallCounts(channels=self._counts.copy(), paused_total=self._paused_total)

        logger.info(f"Ball detected on channel {channel}, total: {counts.total}")
        self._notify_callbacks(counts)
        return counts

    def increment_paused(self) -> BallCounts:
        """Increment the paused count (balls detected while counting is paused).

        Returns:
            Updated BallCounts.
        """
        with self._lock:
            self._paused_total += 1
            counts = BallCounts(channels=self._counts.copy(), paused_total=self._paused_total)

        logger.info(f"Ball detected while paused, paused total: {self._paused_total}")
        self._notify_callbacks(counts)
        return counts

    def reset(self) -> BallCounts:
        """Reset all counts to zero.

        Returns:
            Reset BallCounts (all zeros).
        """
        with self._lock:
            self._counts = [0] * self._num_channels
            self._paused_total = 0
            counts = BallCounts(channels=self._counts.copy(), paused_total=0)

        logger.info("Ball counts reset")
        self._notify_callbacks(counts)
        return counts

    def set_counts(self, channels: list[int]) -> BallCounts:
        """Set counts directly (for restoration from config).

        Args:
            channels: List of count values for each channel.

        Returns:
            Updated BallCounts.
        """
        if len(channels) != self._num_channels:
            raise ValueError(f"Expected {self._num_channels} channels, got {len(channels)}")

        with self._lock:
            self._counts = channels.copy()
            counts = BallCounts(channels=self._counts.copy(), paused_total=self._paused_total)

        logger.info(f"Ball counts set to {counts.total}")
        self._notify_callbacks(counts)
        return counts

    def on_count_change(self, callback: CountChangeCallback) -> None:
        """Register a callback for count changes.

        Args:
            callback: Function to call with updated BallCounts.
        """
        self._callbacks.append(callback)

    def remove_callback(self, callback: CountChangeCallback) -> None:
        """Remove a count change callback.

        Args:
            callback: The callback to remove.
        """
        if callback in self._callbacks:
            self._callbacks.remove(callback)

    def _notify_callbacks(self, counts: BallCounts) -> None:
        """Notify all registered callbacks of count change."""
        for callback in self._callbacks:
            try:
                callback(counts)
            except Exception as e:
                logger.error(f"Error in count change callback: {e}")
