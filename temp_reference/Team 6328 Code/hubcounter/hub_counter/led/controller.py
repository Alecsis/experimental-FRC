"""LED strip controller for visual feedback."""

import asyncio
import logging
import os
import time
from typing import Optional

from ..config.settings import ColorConfig, LEDConfig, ThresholdConfig

logger = logging.getLogger(__name__)

# Check if we're running on a Raspberry Pi
IS_RASPBERRY_PI = os.path.exists("/sys/firmware/devicetree/base/model")
MOCK_HARDWARE = os.environ.get("MOCK_HARDWARE", "0") == "1"

if IS_RASPBERRY_PI and not MOCK_HARDWARE:
    try:
        from rpi_ws281x import Color, PixelStrip

        LED_AVAILABLE = True
    except ImportError:
        logger.warning("rpi_ws281x not available, using mock LED")
        LED_AVAILABLE = False
else:
    LED_AVAILABLE = False

if not LED_AVAILABLE:
    from .mock_led import Color, MockPixelStrip as PixelStrip


class LEDController:
    """Controller for WS2813 LED strips."""

    # LED strip configuration constants
    LED_FREQ_HZ = 800000  # LED signal frequency in Hz
    LED_DMA = 10  # DMA channel for generating signal
    LED_INVERT = False  # True to invert the signal
    LED_CHANNEL = 0  # PWM channel

    def __init__(
        self,
        led_config: LEDConfig,
        threshold_config: ThresholdConfig,
        color_config: ColorConfig,
    ):
        """Initialize the LED controller.

        Args:
            led_config: LED strip configuration.
            threshold_config: Ball count thresholds for color changes.
            color_config: RGB colors for each threshold level.
        """
        self._led_config = led_config
        self._threshold_config = threshold_config
        self._color_config = color_config
        self._strip: Optional[PixelStrip] = None
        self._active = False
        self._pulse_task: Optional[asyncio.Task] = None

        logger.info(
            f"LEDController initialized (hardware={'real' if LED_AVAILABLE else 'mock'})"
        )

    def start(self) -> None:
        """Initialize and start the LED strip."""
        if self._active:
            return

        # Create pixel strip for 2 strips x led_count LEDs each
        total_leds = self._led_config.led_count * 2

        self._strip = PixelStrip(
            total_leds,
            self._led_config.data_pin,
            self.LED_FREQ_HZ,
            self.LED_DMA,
            self.LED_INVERT,
            self._led_config.brightness,
            self.LED_CHANNEL,
        )
        self._strip.begin()

        # Clear all LEDs
        self._clear()

        self._active = True
        logger.info(f"LED strip started with {total_leds} LEDs")

    def stop(self) -> None:
        """Stop and clean up the LED strip."""
        if not self._active:
            return

        if self._pulse_task and not self._pulse_task.done():
            self._pulse_task.cancel()

        self._clear()
        self._active = False
        logger.info("LED strip stopped")

    def _clear(self) -> None:
        """Turn off all LEDs."""
        if not self._strip:
            return

        for i in range(self._strip.numPixels()):
            self._strip.setPixelColor(i, Color(0, 0, 0))
        self._strip.show()

    def get_color_for_count(self, total_count: int) -> tuple[int, int, int]:
        """Get the appropriate color based on total ball count.

        Args:
            total_count: Current total ball count.

        Returns:
            RGB tuple for the appropriate color.
        """
        if total_count >= self._threshold_config.blue:
            return self._color_config.blue
        elif total_count >= self._threshold_config.yellow:
            return self._color_config.yellow
        else:
            return self._color_config.red

    def pulse_sync(
        self, color: tuple[int, int, int], duration_ms: int = 500
    ) -> None:
        """Pulse the LEDs synchronously (blocking).

        Args:
            color: RGB color tuple.
            duration_ms: Pulse duration in milliseconds.
        """
        if not self._strip or not self._active:
            return

        r, g, b = color
        num_leds = self._strip.numPixels()
        steps = 20
        step_delay = duration_ms / 1000.0 / (steps * 2)

        # Fade in
        for step in range(steps):
            brightness = int((step / steps) * 255)
            scaled_r = int(r * brightness / 255)
            scaled_g = int(g * brightness / 255)
            scaled_b = int(b * brightness / 255)

            for i in range(num_leds):
                self._strip.setPixelColor(i, Color(scaled_r, scaled_g, scaled_b))
            self._strip.show()
            time.sleep(step_delay)

        # Fade out
        for step in range(steps, 0, -1):
            brightness = int((step / steps) * 255)
            scaled_r = int(r * brightness / 255)
            scaled_g = int(g * brightness / 255)
            scaled_b = int(b * brightness / 255)

            for i in range(num_leds):
                self._strip.setPixelColor(i, Color(scaled_r, scaled_g, scaled_b))
            self._strip.show()
            time.sleep(step_delay)

        self._clear()

    async def pulse_async(
        self, color: tuple[int, int, int], duration_ms: int = 500
    ) -> None:
        """Pulse the LEDs asynchronously.

        Args:
            color: RGB color tuple.
            duration_ms: Pulse duration in milliseconds.
        """
        if not self._strip or not self._active:
            return

        r, g, b = color
        num_leds = self._strip.numPixels()
        steps = 20
        step_delay = duration_ms / 1000.0 / (steps * 2)

        # Fade in
        for step in range(steps):
            brightness = int((step / steps) * 255)
            scaled_r = int(r * brightness / 255)
            scaled_g = int(g * brightness / 255)
            scaled_b = int(b * brightness / 255)

            for i in range(num_leds):
                self._strip.setPixelColor(i, Color(scaled_r, scaled_g, scaled_b))
            self._strip.show()
            await asyncio.sleep(step_delay)

        # Fade out
        for step in range(steps, 0, -1):
            brightness = int((step / steps) * 255)
            scaled_r = int(r * brightness / 255)
            scaled_g = int(g * brightness / 255)
            scaled_b = int(b * brightness / 255)

            for i in range(num_leds):
                self._strip.setPixelColor(i, Color(scaled_r, scaled_g, scaled_b))
            self._strip.show()
            await asyncio.sleep(step_delay)

        self._clear()

    def set_solid(self, color: tuple[int, int, int]) -> None:
        """Set all LEDs to a solid color.

        Args:
            color: RGB color tuple.
        """
        if not self._strip or not self._active:
            return

        r, g, b = color
        for i in range(self._strip.numPixels()):
            self._strip.setPixelColor(i, Color(r, g, b))
        self._strip.show()
        logger.debug(f"Set solid color: ({r}, {g}, {b})")

    def set_blink(self, color: tuple[int, int, int], loop: asyncio.AbstractEventLoop) -> None:
        """Start a blinking pattern (500ms on, 500ms off).

        Args:
            color: RGB color tuple.
            loop: Asyncio event loop.
        """
        self.stop_pattern()
        self._pulse_task = asyncio.run_coroutine_threadsafe(
            self._blink_loop(color), loop
        )
        logger.debug(f"Started blink pattern with color {color}")

    def set_racing(self, color: tuple[int, int, int], loop: asyncio.AbstractEventLoop) -> None:
        """Start a KITT-style racing/scanner pattern.

        Args:
            color: RGB color tuple.
            loop: Asyncio event loop.
        """
        self.stop_pattern()
        self._pulse_task = asyncio.run_coroutine_threadsafe(
            self._racing_loop(color), loop
        )
        logger.debug(f"Started racing pattern with color {color}")

    def stop_pattern(self) -> None:
        """Cancel any running pattern task and clear LEDs."""
        if self._pulse_task and not self._pulse_task.done():
            self._pulse_task.cancel()
            self._pulse_task = None
        self._clear()

    async def _blink_loop(self, color: tuple[int, int, int]) -> None:
        """Async loop for pulsing/fading pattern.

        Args:
            color: RGB color tuple.
        """
        r, g, b = color
        num_leds = self._strip.numPixels() if self._strip else 0
        steps = 20
        step_delay = 0.5 / steps  # 0.5 seconds for fade in, 0.5 for fade out

        try:
            while True:
                if not self._strip or not self._active:
                    return

                # Fade in
                for step in range(steps):
                    brightness = step / steps
                    scaled_r = int(r * brightness)
                    scaled_g = int(g * brightness)
                    scaled_b = int(b * brightness)

                    for i in range(num_leds):
                        self._strip.setPixelColor(i, Color(scaled_r, scaled_g, scaled_b))
                    self._strip.show()
                    await asyncio.sleep(step_delay)

                # Fade out
                for step in range(steps, 0, -1):
                    brightness = step / steps
                    scaled_r = int(r * brightness)
                    scaled_g = int(g * brightness)
                    scaled_b = int(b * brightness)

                    for i in range(num_leds):
                        self._strip.setPixelColor(i, Color(scaled_r, scaled_g, scaled_b))
                    self._strip.show()
                    await asyncio.sleep(step_delay)

        except asyncio.CancelledError:
            pass

    async def _racing_loop(self, color: tuple[int, int, int]) -> None:
        """Async loop for KITT-style racing/scanner pattern on solid background.

        A white LED sweeps back and forth with a trailing fade on a solid background.

        Args:
            color: RGB color tuple for the solid background LED.
        """
        r, g, b = color
        num_leds = self._strip.numPixels() if self._strip else 0
        if num_leds == 0:
            return

        trail_brightness = [1.0, 0.4, 0.15]  # head + 2 trailing LEDs
        step_delay = 0.04  # 40ms per step for smooth animation
        racing_white_brightness = 90  # Dimmed white foreground

        try:
            while True:
                # Forward sweep
                for pos in range(num_leds):
                    if not self._strip or not self._active:
                        return
                    # Set rgb background
                    for i in range(num_leds):
                        self._strip.setPixelColor(i, Color(r, g, b))
                    # Draw white racing pattern on top
                    for t, brightness in enumerate(trail_brightness):
                        idx = pos - t
                        if 0 <= idx < num_leds:
                            self._strip.setPixelColor(
                                idx,
                                Color(
                                    int(racing_white_brightness * brightness),
                                    int(racing_white_brightness * brightness),
                                    int(racing_white_brightness * brightness),
                                ),
                            )
                    self._strip.show()
                    await asyncio.sleep(step_delay)
                # Reverse sweep
                for pos in range(num_leds - 1, -1, -1):
                    if not self._strip or not self._active:
                        return
                    # Set white background
                    for i in range(num_leds):
                        self._strip.setPixelColor(i, Color(r, g, b))
                    # Draw colored racing pattern on top
                    for t, brightness in enumerate(trail_brightness):
                        idx = pos + t
                        if 0 <= idx < num_leds:
                            self._strip.setPixelColor(
                                idx,
                                Color(
                                    int(racing_white_brightness * brightness),
                                    int(racing_white_brightness * brightness),
                                    int(racing_white_brightness * brightness),
                                ),
                            )
                    self._strip.show()
                    await asyncio.sleep(step_delay)
        except asyncio.CancelledError:
            pass

    def trigger_pulse(self, total_count: int, loop: asyncio.AbstractEventLoop) -> None:
        """Trigger a pulse based on current ball count.

        Args:
            total_count: Current total ball count.
            loop: Asyncio event loop to schedule the pulse on.
        """
        color = self.get_color_for_count(total_count)
        logger.debug(f"Triggering pulse with color {color} for count {total_count}")

        # Cancel any existing pulse
        if self._pulse_task and not self._pulse_task.done():
            self._pulse_task.cancel()

        # Schedule new pulse
        self._pulse_task = asyncio.run_coroutine_threadsafe(
            self.pulse_async(color), loop
        )

    def update_config(
        self,
        threshold_config: Optional[ThresholdConfig] = None,
        color_config: Optional[ColorConfig] = None,
    ) -> None:
        """Update LED configuration.

        Args:
            threshold_config: New threshold configuration.
            color_config: New color configuration.
        """
        if threshold_config:
            self._threshold_config = threshold_config
        if color_config:
            self._color_config = color_config
        logger.info("LED configuration updated")

    @property
    def is_active(self) -> bool:
        """Check if LED controller is active."""
        return self._active
