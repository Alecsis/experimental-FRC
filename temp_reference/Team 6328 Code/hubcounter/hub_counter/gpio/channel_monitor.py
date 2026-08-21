"""GPIO channel monitoring for ball detection."""

import logging
import os
from typing import Callable, Optional

from ..config.settings import GPIOConfig
from ..core.events import EventBus, EventType

logger = logging.getLogger(__name__)

# Check if we're running on a Raspberry Pi
IS_RASPBERRY_PI = os.path.exists("/sys/firmware/devicetree/base/model")
MOCK_HARDWARE = os.environ.get("MOCK_HARDWARE", "0") == "1"

if IS_RASPBERRY_PI and not MOCK_HARDWARE:
    try:
        from gpiozero import Button

        GPIO_AVAILABLE = True
    except ImportError:
        logger.warning("gpiozero not available, using mock GPIO")
        GPIO_AVAILABLE = False
else:
    GPIO_AVAILABLE = False

if not GPIO_AVAILABLE:
    from .mock_gpio import MockButton as Button, gpio_simulator


class ChannelMonitor:
    """Monitor GPIO pins for ball detection events."""

    def __init__(
        self,
        config: GPIOConfig,
        event_bus: EventBus,
        on_ball_detected: Optional[Callable[[int], None]] = None,
    ):
        """Initialize the channel monitor.

        Args:
            config: GPIO configuration.
            event_bus: Event bus for emitting events.
            on_ball_detected: Optional direct callback when ball detected.
        """
        self._config = config
        self._event_bus = event_bus
        self._on_ball_detected = on_ball_detected
        self._buttons: list = []
        self._running = False

        logger.info(
            f"ChannelMonitor initialized (hardware={'real' if GPIO_AVAILABLE else 'mock'})"
        )

    def start(self) -> None:
        """Start monitoring GPIO channels."""
        if self._running:
            return

        bounce_time = self._config.debounce_ms / 1000.0  # Convert to seconds

        for channel_idx, pin in enumerate(self._config.channel_pins):
            # Create button with appropriate pull resistor configuration
            # When pull_up=True: internal pull-up enabled, button press goes LOW (active low)
            # When pull_up=False: internal pull-down enabled, button press goes HIGH (active high)
            # Note: Don't specify active_state when using pull resistors - gpiozero infers it
            button = Button(
                pin,
                pull_up=self._config.active_low,  # pull_up=True for active-low sensors
                bounce_time=bounce_time,
            )

            # Set up callback for this channel
            button.when_pressed = self._create_handler(channel_idx)

            self._buttons.append(button)
            logger.debug(f"Monitoring pin {pin} as channel {channel_idx}")

            # Register with simulator if using mock GPIO
            if not GPIO_AVAILABLE:
                gpio_simulator.register_button(button)

        self._running = True
        logger.info(f"Started monitoring {len(self._buttons)} channels")

    def stop(self) -> None:
        """Stop monitoring GPIO channels."""
        if not self._running:
            return

        for button in self._buttons:
            button.close()

        self._buttons.clear()
        self._running = False
        logger.info("Stopped channel monitoring")

    def _create_handler(self, channel: int) -> Callable[[], None]:
        """Create a handler for a specific channel.

        Args:
            channel: Channel index.

        Returns:
            Handler function for the button press.
        """

        def handler():
            logger.debug(f"Ball detected on channel {channel}")

            # Emit event (sync since this is from GPIO callback thread)
            self._event_bus.emit_sync(
                EventType.BALL_DETECTED, {"channel": channel}
            )

            # Call direct callback if provided
            if self._on_ball_detected:
                self._on_ball_detected(channel)

        return handler

    @property
    def is_running(self) -> bool:
        """Check if monitoring is active."""
        return self._running

    def simulate_ball(self, channel: int) -> None:
        """Simulate a ball detection for testing.

        Args:
            channel: Channel to simulate ball on.
        """
        if not GPIO_AVAILABLE and 0 <= channel < len(self._buttons):
            self._buttons[channel].simulate_press()
        else:
            # For real GPIO, just call the handler directly for testing
            handler = self._create_handler(channel)
            handler()
