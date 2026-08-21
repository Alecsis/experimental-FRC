"""Event bus for component communication."""

import asyncio
import logging
from collections import defaultdict
from enum import Enum, auto
from typing import Any, Callable, Coroutine

logger = logging.getLogger(__name__)


class EventType(Enum):
    """Event types for the application."""

    BALL_DETECTED = auto()  # Payload: {"channel": int}
    COUNT_CHANGED = auto()  # Payload: BallCounts
    CONFIG_UPDATED = auto()  # Payload: Settings
    NT_CONNECTED = auto()  # Payload: {"connected": bool}
    MATCH_INFO_UPDATED = auto()  # Payload: MatchInfo
    EXTERNAL_STATE_CHANGED = auto()  # Payload: {"is_external": bool, "pattern": int, "color": str}


# Type alias for event handlers
SyncHandler = Callable[[Any], None]
AsyncHandler = Callable[[Any], Coroutine[Any, Any, None]]
Handler = SyncHandler | AsyncHandler


class EventBus:
    """Simple event bus for decoupled component communication."""

    def __init__(self):
        """Initialize the event bus."""
        self._handlers: dict[EventType, list[Handler]] = defaultdict(list)
        self._loop: asyncio.AbstractEventLoop | None = None

    def set_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Set the asyncio event loop for sync-to-async bridging.

        Args:
            loop: The asyncio event loop to use.
        """
        self._loop = loop

    def subscribe(self, event_type: EventType, handler: Handler) -> None:
        """Subscribe to an event type.

        Args:
            event_type: The type of event to subscribe to.
            handler: Callback function (sync or async) to call when event fires.
        """
        self._handlers[event_type].append(handler)
        logger.debug(f"Handler subscribed to {event_type.name}")

    def unsubscribe(self, event_type: EventType, handler: Handler) -> None:
        """Unsubscribe from an event type.

        Args:
            event_type: The type of event to unsubscribe from.
            handler: The handler to remove.
        """
        if handler in self._handlers[event_type]:
            self._handlers[event_type].remove(handler)
            logger.debug(f"Handler unsubscribed from {event_type.name}")

    async def emit_async(self, event_type: EventType, payload: Any = None) -> None:
        """Emit an event asynchronously.

        Args:
            event_type: The type of event to emit.
            payload: Data to pass to handlers.
        """
        handlers = self._handlers.get(event_type, [])
        logger.debug(f"Emitting {event_type.name} to {len(handlers)} handlers")

        for handler in handlers:
            try:
                if asyncio.iscoroutinefunction(handler):
                    await handler(payload)
                else:
                    handler(payload)
            except Exception as e:
                logger.error(f"Error in handler for {event_type.name}: {e}")

    def emit_sync(self, event_type: EventType, payload: Any = None) -> None:
        """Emit an event from a synchronous context (e.g., GPIO callback).

        This schedules the event handlers to run on the event loop.

        Args:
            event_type: The type of event to emit.
            payload: Data to pass to handlers.
        """
        if self._loop is None:
            logger.warning(f"No event loop set, cannot emit {event_type.name}")
            return

        if self._loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self.emit_async(event_type, payload), self._loop
            )
        else:
            logger.warning(f"Event loop not running, cannot emit {event_type.name}")


# Global event bus instance
event_bus = EventBus()
