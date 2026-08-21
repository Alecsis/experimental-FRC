"""Mock LED strip for testing on non-Pi systems."""

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class MockPixelStrip:
    """Mock rpi_ws281x PixelStrip for testing."""

    def __init__(
        self,
        num: int,
        pin: int,
        freq_hz: int = 800000,
        dma: int = 10,
        invert: bool = False,
        brightness: int = 255,
        channel: int = 0,
        strip_type: Optional[int] = None,
    ):
        """Initialize mock pixel strip.

        Args:
            num: Number of LEDs.
            pin: GPIO pin number.
            freq_hz: LED signal frequency.
            dma: DMA channel.
            invert: Invert signal.
            brightness: Initial brightness (0-255).
            channel: PWM channel.
            strip_type: LED strip type.
        """
        self._num_pixels = num
        self._pin = pin
        self._brightness = brightness
        self._pixels = [(0, 0, 0)] * num
        logger.debug(f"MockPixelStrip created: {num} LEDs on pin {pin}")

    def begin(self) -> None:
        """Initialize the strip."""
        logger.debug("MockPixelStrip initialized")

    def show(self) -> None:
        """Update the LEDs with current pixel values."""
        # Log the current state for debugging
        active_pixels = [
            (i, self._pixels[i])
            for i in range(self._num_pixels)
            if self._pixels[i] != (0, 0, 0)
        ]
        if active_pixels:
            logger.debug(f"LED update: {active_pixels}")

    def setPixelColor(self, n: int, color: int) -> None:
        """Set pixel color using packed RGB value.

        Args:
            n: Pixel index.
            color: Packed RGB color (0xRRGGBB).
        """
        if 0 <= n < self._num_pixels:
            r = (color >> 16) & 0xFF
            g = (color >> 8) & 0xFF
            b = color & 0xFF
            self._pixels[n] = (r, g, b)

    def setPixelColorRGB(self, n: int, r: int, g: int, b: int) -> None:
        """Set pixel color using RGB values.

        Args:
            n: Pixel index.
            r: Red component (0-255).
            g: Green component (0-255).
            b: Blue component (0-255).
        """
        if 0 <= n < self._num_pixels:
            self._pixels[n] = (r, g, b)

    def getPixelColor(self, n: int) -> int:
        """Get pixel color as packed RGB value.

        Args:
            n: Pixel index.

        Returns:
            Packed RGB color value.
        """
        if 0 <= n < self._num_pixels:
            r, g, b = self._pixels[n]
            return (r << 16) | (g << 8) | b
        return 0

    def numPixels(self) -> int:
        """Get number of pixels."""
        return self._num_pixels

    def setBrightness(self, brightness: int) -> None:
        """Set strip brightness.

        Args:
            brightness: Brightness level (0-255).
        """
        self._brightness = max(0, min(255, brightness))

    def getBrightness(self) -> int:
        """Get current brightness."""
        return self._brightness

    def getPixels(self) -> list:
        """Get all pixel values."""
        return self._pixels.copy()


def Color(red: int, green: int, blue: int, white: int = 0) -> int:
    """Create a packed color value.

    Args:
        red: Red component (0-255).
        green: Green component (0-255).
        blue: Blue component (0-255).
        white: White component for RGBW strips (0-255).

    Returns:
        Packed color value.
    """
    return (white << 24) | (red << 16) | (green << 8) | blue
