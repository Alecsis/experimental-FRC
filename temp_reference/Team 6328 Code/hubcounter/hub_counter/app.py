"""Main application orchestrator."""

import asyncio
import logging
import signal
from pathlib import Path
from typing import Optional

import uvicorn

from .config.settings import Settings, load_settings, save_settings
from .core.events import EventBus, EventType, event_bus
from .core.models import BallCounts
from .gpio.ball_counter import BallCounter
from .gpio.channel_monitor import ChannelMonitor
from .led.controller import LEDController
from .network.nt_client import NetworkTablesClient
from .web.routes.api import broadcast_counts, broadcast_external_state, broadcast_nt_status, set_dependencies
from .web.server import create_app

logger = logging.getLogger(__name__)


class HubCounterApp:
    """Main application orchestrating all components."""

    def __init__(self, config_path: Optional[Path] = None):
        """Initialize the application.

        Args:
            config_path: Path to configuration file.
        """
        self._config_path = config_path or Path("config.yaml")
        self._settings = load_settings(self._config_path)

        # Components
        self._event_bus = event_bus
        self._ball_counter = BallCounter(num_channels=4)
        self._channel_monitor: Optional[ChannelMonitor] = None
        self._led_controller: Optional[LEDController] = None
        self._nt_client: Optional[NetworkTablesClient] = None

        # State
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._shutdown_event = asyncio.Event()
        self._server: Optional[uvicorn.Server] = None
        self._external_poll_task: Optional[asyncio.Task] = None
        self._is_external = False
        self._external_dismissed = False
        self._last_robot_external = False
        self._last_external_pattern = -1
        self._last_external_color = ""
        self._pause_counting = False
        self._last_pause_counting = False

        logger.info("HubCounterApp initialized")

    def _setup_components(self) -> None:
        """Initialize all hardware and network components."""
        # Set up event loop for sync-to-async bridging
        self._loop = asyncio.get_event_loop()
        self._event_bus.set_loop(self._loop)

        # Initialize channel monitor
        self._channel_monitor = ChannelMonitor(
            config=self._settings.gpio,
            event_bus=self._event_bus,
            on_ball_detected=self._on_ball_detected,
        )

        # Initialize LED controller
        self._led_controller = LEDController(
            led_config=self._settings.led,
            threshold_config=self._settings.thresholds,
            color_config=self._settings.colors,
        )

        # Initialize NetworkTables client
        self._nt_client = NetworkTablesClient(
            config=self._settings.network,
            event_bus=self._event_bus,
        )

        # Register ball counter callback for event bus updates
        self._ball_counter.on_count_change(self._on_count_changed)

        # Subscribe to NT connection changes for WebSocket broadcasting
        self._event_bus.subscribe(
            EventType.NT_CONNECTED, self._on_nt_connection_changed
        )

        logger.info("All components initialized")

    def _on_ball_detected(self, channel: int) -> None:
        """Handle ball detection from GPIO.

        Args:
            channel: Channel where ball was detected.
        """
        # Check if counting is paused (only applies during external control)
        if self._is_external and self._pause_counting:
            logger.debug(f"Ball detected on channel {channel} but counting is paused — incrementing paused count")
            counts = self._ball_counter.increment_paused()
            if self._nt_client:
                self._nt_client.publish_counts(counts)
            if self._loop:
                asyncio.run_coroutine_threadsafe(broadcast_counts(counts), self._loop)
            return

        counts = self._ball_counter.increment(channel)

        # Trigger LED pulse only when not in external control mode
        if self._led_controller and self._loop and not self._is_external:
            self._led_controller.trigger_pulse(counts.total, self._loop)

        # Publish to NetworkTables
        if self._nt_client:
            self._nt_client.publish_counts(counts)

    def _on_count_changed(self, counts: BallCounts) -> None:
        """Handle count changes.

        Args:
            counts: Updated ball counts.
        """
        # Schedule WebSocket broadcast
        if self._loop:
            asyncio.run_coroutine_threadsafe(
                broadcast_counts(counts), self._loop
            )

    async def _on_nt_connection_changed(self, payload: dict) -> None:
        """Handle NT connection state changes.

        Args:
            payload: Event payload with 'connected' bool.
        """
        connected = payload.get("connected", False)

        # Revert external control on disconnect
        if not connected and self._is_external:
            logger.info("NT disconnected, reverting external control")
            self._is_external = False
            self._external_dismissed = False
            self._last_robot_external = False
            if self._led_controller:
                self._led_controller.stop_pattern()
            await broadcast_external_state(False, "", "", False)

        await broadcast_nt_status(connected)

    async def _poll_external_state(self) -> None:
        """Poll NetworkTables for external control state changes every 100ms."""
        PATTERN_NAMES = {0: "Solid", 1: "Blink", 2: "Racing"}

        while True:
            try:
                await asyncio.sleep(0.1)

                if not self._nt_client:
                    continue

                # Check for reset request
                if self._nt_client.get_reset_requested():
                    logger.info("Reset requested via NetworkTables")
                    counts = self._ball_counter.reset()
                    self._nt_client.publish_counts(counts)
                    self._nt_client.acknowledge_reset()
                    await broadcast_counts(counts)

                state = self._nt_client.get_external_state()
                robot_external = state["is_external"]
                pattern = state["led_pattern"]
                color_hex = state["led_color"]
                self._pause_counting = state["pause_counting"]

                # Detect rising edge on robot's IsExternal (false->true)
                if robot_external and not self._last_robot_external:
                    if not self._external_dismissed:
                        self._is_external = True
                        self._last_external_pattern = -1  # force re-apply
                        self._last_external_color = ""
                        logger.info("Entering external control mode")
                        await broadcast_external_state(
                            True, PATTERN_NAMES.get(pattern, "Unknown"), color_hex, self._pause_counting
                        )

                # Detect falling edge on robot's IsExternal (true->false)
                if not robot_external and self._last_robot_external:
                    self._external_dismissed = False
                    if self._is_external:
                        self._is_external = False
                        if self._led_controller:
                            self._led_controller.stop_pattern()
                        self._last_external_pattern = -1
                        self._last_external_color = ""
                        logger.info("Exiting external control mode")
                        await broadcast_external_state(False, "", "", False)

                self._last_robot_external = robot_external

                # Broadcast pause_counting changes while in external mode
                if self._is_external and self._pause_counting != self._last_pause_counting:
                    self._last_pause_counting = self._pause_counting
                    await broadcast_external_state(
                        True,
                        PATTERN_NAMES.get(pattern, "Unknown"),
                        color_hex,
                        self._pause_counting,
                    )

                # Apply LED pattern/color if in external mode
                if self._is_external and self._led_controller and self._loop:
                    if pattern != self._last_external_pattern or color_hex != self._last_external_color:
                        # Parse hex color
                        rgb = self._parse_hex_color(color_hex)
                        self._last_external_pattern = pattern
                        self._last_external_color = color_hex

                        if pattern == 0:  # Solid
                            self._led_controller.stop_pattern()
                            self._led_controller.set_solid(rgb)
                        elif pattern == 1:  # Blink
                            self._led_controller.set_blink(rgb, self._loop)
                        elif pattern == 2:  # Racing
                            self._led_controller.set_racing(rgb, self._loop)

                        logger.info(
                            f"External LED: pattern={PATTERN_NAMES.get(pattern, pattern)}, color={color_hex}"
                        )
                        await broadcast_external_state(
                            True, PATTERN_NAMES.get(pattern, "Unknown"), color_hex, self._pause_counting
                        )

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error in external state poll: {e}", exc_info=True)

    @staticmethod
    def _parse_hex_color(hex_color: str) -> tuple[int, int, int]:
        """Parse an HTML hex color string to an RGB tuple.

        Args:
            hex_color: Color string like '#ff0000'.

        Returns:
            RGB tuple.
        """
        hex_color = hex_color.lstrip("#")
        if len(hex_color) != 6:
            return (0, 0, 0)
        try:
            r = int(hex_color[0:2], 16)
            g = int(hex_color[2:4], 16)
            b = int(hex_color[4:6], 16)
            return (r, g, b)
        except ValueError:
            return (0, 0, 0)

    async def dismiss_external(self) -> None:
        """Dismiss external control mode (UI override).

        Sets _external_dismissed so the robot's IsExternal is ignored
        until it toggles off and back on.
        """
        self._external_dismissed = True
        self._is_external = False
        self._last_external_pattern = -1
        self._last_external_color = ""
        if self._led_controller:
            self._led_controller.stop_pattern()

        # Tell the robot control was dismissed
        if self._nt_client:
            self._nt_client.set_external(False)

        logger.info("External control dismissed by user")
        await broadcast_external_state(False, "", "", False)

    def _save_settings(self, settings: Settings) -> None:
        """Save settings to file.

        Args:
            settings: Settings to save.
        """
        self._settings = settings
        save_settings(settings, self._config_path)
        logger.info("Settings saved")

    async def _run_server(self) -> None:
        """Run the web server."""
        app = create_app()

        # Set up API dependencies
        set_dependencies(
            ball_counter=self._ball_counter,
            nt_client=self._nt_client,
            led_controller=self._led_controller,
            settings=self._settings,
            save_settings_fn=self._save_settings,
            app=self,
        )

        config = uvicorn.Config(
            app,
            host=self._settings.web.host,
            port=self._settings.web.port,
            log_level="info",
        )
        self._server = uvicorn.Server(config)

        logger.info(
            f"Starting web server on {self._settings.web.host}:{self._settings.web.port}"
        )
        await self._server.serve()

    def _setup_signal_handlers(self) -> None:
        """Set up signal handlers for graceful shutdown."""

        def signal_handler(sig, frame):
            logger.info(f"Received signal {sig}, initiating shutdown...")
            self._shutdown_event.set()
            if self._server:
                self._server.should_exit = True

        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

    async def run(self) -> None:
        """Run the application."""
        self._setup_components()
        self._setup_signal_handlers()

        # Start hardware components
        if self._channel_monitor:
            self._channel_monitor.start()

        if self._led_controller:
            self._led_controller.start()

        if self._nt_client:
            self._nt_client.start()

        # Start external control polling
        self._external_poll_task = asyncio.create_task(self._poll_external_state())

        logger.info("Hub Counter started")

        try:
            # Run web server (this blocks until shutdown)
            await self._run_server()
        finally:
            await self.shutdown()

    async def shutdown(self) -> None:
        """Clean up and shut down all components."""
        logger.info("Shutting down...")

        # Stop external poll task
        if self._external_poll_task and not self._external_poll_task.done():
            self._external_poll_task.cancel()
            try:
                await self._external_poll_task
            except asyncio.CancelledError:
                pass

        # Stop hardware components
        if self._channel_monitor:
            self._channel_monitor.stop()

        if self._led_controller:
            self._led_controller.stop()

        if self._nt_client:
            self._nt_client.stop()

        logger.info("Hub Counter stopped")


def run_app(config_path: Optional[Path] = None) -> None:
    """Run the Hub Counter application.

    Args:
        config_path: Path to configuration file.
    """
    app = HubCounterApp(config_path)
    asyncio.run(app.run())
