"""Mock GPIO for testing on non-Pi systems."""

import logging
import threading
import time
from typing import Callable, Optional

logger = logging.getLogger(__name__)


class MockButton:
    """Mock gpiozero Button for testing."""

    def __init__(
        self,
        pin: int,
        pull_up: bool = True,
        active_state: Optional[bool] = None,
        bounce_time: Optional[float] = None,
    ):
        """Initialize mock button.

        Args:
            pin: GPIO pin number.
            pull_up: Whether to use pull-up resistor.
            active_state: Active state (True=high, False=low, None=auto).
            bounce_time: Debounce time in seconds.
        """
        self.pin = pin
        self.pull_up = pull_up
        self.active_state = active_state
        self.bounce_time = bounce_time
        self._when_pressed: Optional[Callable] = None
        self._when_released: Optional[Callable] = None
        self._is_pressed = False
        logger.debug(f"MockButton created for pin {pin}")

    @property
    def when_pressed(self) -> Optional[Callable]:
        """Get press handler."""
        return self._when_pressed

    @when_pressed.setter
    def when_pressed(self, callback: Optional[Callable]) -> None:
        """Set press handler."""
        self._when_pressed = callback

    @property
    def when_released(self) -> Optional[Callable]:
        """Get release handler."""
        return self._when_released

    @when_released.setter
    def when_released(self, callback: Optional[Callable]) -> None:
        """Set release handler."""
        self._when_released = callback

    @property
    def is_pressed(self) -> bool:
        """Check if button is pressed."""
        return self._is_pressed

    def simulate_press(self) -> None:
        """Simulate a button press (for testing)."""
        self._is_pressed = True
        if self._when_pressed:
            self._when_pressed()

    def simulate_release(self) -> None:
        """Simulate a button release (for testing)."""
        self._is_pressed = False
        if self._when_released:
            self._when_released()

    def simulate_click(self, hold_time: float = 0.1) -> None:
        """Simulate a button click (press and release).

        Args:
            hold_time: Time to hold button in seconds.
        """
        self.simulate_press()
        time.sleep(hold_time)
        self.simulate_release()

    def close(self) -> None:
        """Clean up resources."""
        logger.debug(f"MockButton for pin {self.pin} closed")


class MockGPIOSimulator:
    """Simulator for generating test GPIO events."""

    def __init__(self):
        """Initialize the simulator."""
        self._buttons: dict[int, MockButton] = {}
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def register_button(self, button: MockButton) -> None:
        """Register a button for simulation.

        Args:
            button: MockButton instance to register.
        """
        self._buttons[button.pin] = button
        logger.debug(f"Registered button on pin {button.pin} for simulation")

    def trigger_pin(self, pin: int) -> None:
        """Trigger a button press on a specific pin.

        Args:
            pin: GPIO pin number to trigger.
        """
        if pin in self._buttons:
            self._buttons[pin].simulate_press()
        else:
            logger.warning(f"No button registered for pin {pin}")

    def start_random_simulation(
        self, interval_range: tuple[float, float] = (0.5, 2.0)
    ) -> None:
        """Start random button presses for testing.

        Args:
            interval_range: Min and max interval between presses in seconds.
        """
        import random

        self._running = True

        def simulation_loop():
            while self._running:
                if self._buttons:
                    pin = random.choice(list(self._buttons.keys()))
                    self.trigger_pin(pin)
                interval = random.uniform(*interval_range)
                time.sleep(interval)

        self._thread = threading.Thread(target=simulation_loop, daemon=True)
        self._thread.start()
        logger.info("Started random GPIO simulation")

    def stop_simulation(self) -> None:
        """Stop the random simulation."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        logger.info("Stopped GPIO simulation")


# Global simulator instance for testing
gpio_simulator = MockGPIOSimulator()
