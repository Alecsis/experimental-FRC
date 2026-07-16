"""NetworkTables client for FRC integration."""

import logging
import threading
from typing import Callable, Optional

from ..config.settings import NetworkConfig
from ..core.events import EventBus, EventType
from ..core.models import BallCounts, MatchInfo

logger = logging.getLogger(__name__)

try:
    import ntcore

    NT_AVAILABLE = True
    logger.info(f"pyntcore loaded successfully, version info: {ntcore.__file__ if hasattr(ntcore, '__file__') else 'unknown'}")
except ImportError as e:
    logger.warning(f"pyntcore not available, NetworkTables disabled: {e}")
    NT_AVAILABLE = False


class NetworkTablesClient:
    """NetworkTables 4 client for FRC robot communication."""

    def __init__(
        self,
        config: NetworkConfig,
        event_bus: EventBus,
        on_match_info: Optional[Callable[[MatchInfo], None]] = None,
    ):
        """Initialize the NetworkTables client.

        Args:
            config: Network configuration.
            event_bus: Event bus for emitting events.
            on_match_info: Optional callback for match info updates.
        """
        self._config = config
        self._event_bus = event_bus
        self._on_match_info = on_match_info

        self._inst: Optional["ntcore.NetworkTableInstance"] = None
        self._total_count_pub = None
        self._paused_total_count_pub = None
        self._channel_counts_pub = None
        self._is_external_pub = None
        self._led_pattern_pub = None
        self._led_color_pub = None
        self._reset_counts_pub = None
        self._pause_counting_pub = None
        self._match_time_sub = None
        self._is_auto_sub = None
        self._is_teleop_sub = None
        self._connected = False
        self._is_external_sub = None
        self._led_pattern_sub = None
        self._led_color_sub = None
        self._reset_counts_sub = None
        self._pause_counting_sub = None

        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_monitor = threading.Event()
        self._conn_listener_handle = None

        logger.info(
            f"NetworkTablesClient initialized (available={NT_AVAILABLE}, team={config.team_number}, robot_address={config.robot_address})"
        )

    def start(self) -> None:
        """Start the NetworkTables client and connect to robot."""
        if not NT_AVAILABLE:
            logger.warning("NetworkTables not available, skipping start")
            return

        logger.debug("Getting default NetworkTableInstance...")
        self._inst = ntcore.NetworkTableInstance.getDefault()

        # Set up identity
        team_number = self._config.team_number
        logger.debug(f"Setting server team to {team_number}")
        self._inst.setServerTeam(team_number)

        # Calculate expected addresses for team number
        # NT4 uses mDNS and falls back to static IPs:
        # roboRIO: 10.TE.AM.2 (e.g., 10.64.28.2 for team 6328)
        team_high = team_number // 100
        team_low = team_number % 100
        expected_ip = f"10.{team_high}.{team_low}.2"
        logger.info(f"Default roboRIO address for team {team_number}: {expected_ip}")

        # If explicit address is provided, use it
        if self._config.robot_address:
            logger.info(f"Using explicit robot address: {self._config.robot_address}")
            self._inst.setServer(self._config.robot_address)
        else:
            logger.info(f"Using team number lookup (mDNS + {expected_ip})")

        # Start client
        logger.debug("Starting NT4 client as 'HubCounter'...")
        self._inst.startClient4("HubCounter")
        logger.info("NT4 client started")

        # Create publishers for ball counts
        logger.debug("Setting up HubCounter table publishers...")
        hub_table = self._inst.getTable("HubCounter")

        self._total_count_pub = hub_table.getIntegerTopic("TotalCount").publish()
        self._paused_total_count_pub = hub_table.getIntegerTopic("PausedTotalCount").publish()
        self._channel_counts_pub = hub_table.getIntegerArrayTopic(
            "ChannelCounts"
        ).publish()

        # Publish initial counts so topics appear immediately
        self._total_count_pub.set(0)
        self._paused_total_count_pub.set(0)
        self._channel_counts_pub.set([0, 0, 0, 0])
        logger.debug("Publishers created for TotalCount, PausedTotalCount, and ChannelCounts with initial values")

        # Create publishers for external control state and publish initial values
        self._is_external_pub = hub_table.getBooleanTopic("IsExternal").publish()
        self._led_pattern_pub = hub_table.getIntegerTopic("Led/Pattern").publish()
        self._led_color_pub = hub_table.getStringTopic("Led/Color").publish()
        self._reset_counts_pub = hub_table.getBooleanTopic("ResetCounts").publish()
        self._pause_counting_pub = hub_table.getBooleanTopic("PauseCounting").publish()

        # Publish initial values
        self._is_external_pub.set(False)
        self._led_pattern_pub.set(0)
        self._led_color_pub.set("#000000")
        self._reset_counts_pub.set(False)
        self._pause_counting_pub.set(False)
        logger.debug("Publishers created for external control topics with initial values")

        # Subscribe to FMS info for match data
        logger.debug("Setting up FMSInfo table subscribers...")
        fms_table = self._inst.getTable("FMSInfo")
        self._match_time_sub = fms_table.getDoubleTopic("MatchTime").subscribe(0.0)
        self._is_auto_sub = fms_table.getBooleanTopic("IsAutonomous").subscribe(False)
        self._is_teleop_sub = fms_table.getBooleanTopic("IsTeleop").subscribe(False)
        logger.debug("Subscribers created for FMSInfo topics")

        # Subscribe to HubCounter table for external control
        logger.debug("Setting up HubCounter table subscribers for external control...")
        self._is_external_sub = hub_table.getBooleanTopic("IsExternal").subscribe(False)
        self._led_pattern_sub = hub_table.getIntegerTopic("Led/Pattern").subscribe(0)
        self._led_color_sub = hub_table.getStringTopic("Led/Color").subscribe("#000000")
        self._reset_counts_sub = hub_table.getBooleanTopic("ResetCounts").subscribe(False)
        self._pause_counting_sub = hub_table.getBooleanTopic("PauseCounting").subscribe(False)
        logger.debug("Subscribers created for HubCounter external control topics")

        # Add connection listener
        logger.debug("Adding connection listener...")
        self._conn_listener_handle = self._inst.addConnectionListener(True, self._on_connection_change)

        # Start connection status monitor thread
        self._stop_monitor.clear()
        self._monitor_thread = threading.Thread(
            target=self._connection_monitor,
            name="NT-ConnectionMonitor",
            daemon=True,
        )
        self._monitor_thread.start()
        logger.debug("Connection monitor thread started")

        logger.info(
            f"NetworkTables client started, connecting to team {team_number}"
        )

    def stop(self) -> None:
        """Stop the NetworkTables client."""
        if not NT_AVAILABLE or not self._inst:
            return

        logger.debug("Stopping NetworkTables client...")

        # Stop monitor thread
        self._stop_monitor.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2.0)
            logger.debug("Connection monitor thread stopped")

        # Close publishers
        if self._total_count_pub:
            self._total_count_pub.close()
        if self._paused_total_count_pub:
            self._paused_total_count_pub.close()
        if self._channel_counts_pub:
            self._channel_counts_pub.close()
        if self._is_external_pub:
            self._is_external_pub.close()
        if self._led_pattern_pub:
            self._led_pattern_pub.close()
        if self._led_color_pub:
            self._led_color_pub.close()
        if self._reset_counts_pub:
            self._reset_counts_pub.close()
        if self._pause_counting_pub:
            self._pause_counting_pub.close()

        # Close subscribers
        if self._match_time_sub:
            self._match_time_sub.close()
        if self._is_auto_sub:
            self._is_auto_sub.close()
        if self._is_teleop_sub:
            self._is_teleop_sub.close()
        if self._is_external_sub:
            self._is_external_sub.close()
        if self._led_pattern_sub:
            self._led_pattern_sub.close()
        if self._led_color_sub:
            self._led_color_sub.close()
        if self._reset_counts_sub:
            self._reset_counts_sub.close()
        if self._pause_counting_sub:
            self._pause_counting_sub.close()

        # Remove connection listener
        if getattr(self, "_conn_listener_handle", None) is not None:
            self._inst.removeListener(self._conn_listener_handle)
            self._conn_listener_handle = None

        self._inst.stopClient()
        self._connected = False
        self._event_bus.emit_sync(EventType.NT_CONNECTED, {"connected": False})
        logger.info("NetworkTables client stopped")

    def _connection_monitor(self) -> None:
        """Background thread to periodically log connection status."""
        logger.debug("Connection monitor started")
        check_interval = 5.0  # seconds between status checks

        while not self._stop_monitor.wait(timeout=check_interval):
            if not self._inst:
                continue

            try:
                # Get connection info
                connections = self._inst.getConnections()
                is_connected = self._inst.isConnected()

                if is_connected:
                    logger.debug(f"NT connected: {len(connections)} connection(s)")
                    for conn in connections:
                        remote_id = conn.remote_id if hasattr(conn, 'remote_id') else 'unknown'
                        remote_ip = conn.remote_ip if hasattr(conn, 'remote_ip') else 'unknown'
                        remote_port = conn.remote_port if hasattr(conn, 'remote_port') else 'unknown'
                        logger.debug(f"  - Connected to {remote_id} at {remote_ip}:{remote_port}")
                else:
                    logger.debug(f"NT not connected (isConnected={is_connected}, connections={len(connections)})")

                    # Log server configuration
                    servers = self._inst.getServers() if hasattr(self._inst, 'getServers') else []
                    if servers:
                        logger.debug(f"  Configured servers: {servers}")
                    else:
                        logger.debug(f"  Using team number {self._config.team_number} for server discovery")

            except Exception as e:
                logger.warning(f"Error in connection monitor: {e}")

    def _on_connection_change(self, event: "ntcore.Event") -> None:
        """Handle connection state changes.
        Args:
            event: NetworkTables connection event.
        """
        # Log all event attributes for debugging
        event_attrs = {attr: getattr(event, attr, None) for attr in dir(event) if not attr.startswith('_')}
        logger.debug(f"Connection event received: {event_attrs}")

        # Correctly parse NT4 event data
        if hasattr(event, "data") and hasattr(event.data, "connected"):
            connected = event.data.connected
        else:
            connected = self._inst.isConnected()
        
        self._connected = connected
        self._event_bus.emit_sync(EventType.NT_CONNECTED, {"connected": connected})

        if connected:
            # Try to get connection details
            conn_info = ""
            if hasattr(event, "data") and event.data:
                conn_data = event.data
                if hasattr(conn_data, "remote_id"):
                    conn_info = f" (remote: {conn_data.remote_id})"
                elif hasattr(conn_data, "remote_ip"):
                    conn_info = f" (ip: {conn_data.remote_ip})"
            logger.info(f"Connected to NetworkTables server{conn_info}")
        else:
            logger.warning("Disconnected from NetworkTables server")

    def publish_counts(self, counts: BallCounts) -> None:
        """Publish ball counts to NetworkTables.

        Args:
            counts: Current ball counts.
        """
        if not NT_AVAILABLE or not self._inst:
            logger.debug(f"Skipping publish: NT_AVAILABLE={NT_AVAILABLE}, inst={self._inst is not None}")
            return

        try:
            if self._total_count_pub:
                self._total_count_pub.set(counts.total)
            if self._paused_total_count_pub:
                self._paused_total_count_pub.set(counts.paused_total)
            if self._channel_counts_pub:
                self._channel_counts_pub.set(counts.channels)

            is_connected = self._inst.isConnected() if self._inst else False
            logger.debug(f"Published counts: total={counts.total}, paused_total={counts.paused_total}, connected={is_connected}")
        except Exception as e:
            logger.error(f"Error publishing counts: {e}", exc_info=True)

    def get_match_info(self) -> MatchInfo:
        """Get current match information from NetworkTables.

        Returns:
            Current match info.
        """
        if not NT_AVAILABLE or not self._inst:
            return MatchInfo()

        try:
            match_time = self._match_time_sub.get() if self._match_time_sub else 0.0
            is_auto = self._is_auto_sub.get() if self._is_auto_sub else False
            is_teleop = self._is_teleop_sub.get() if self._is_teleop_sub else False

            return MatchInfo(
                match_time=match_time,
                is_autonomous=is_auto,
                is_teleop=is_teleop,
                is_disabled=not (is_auto or is_teleop),
            )
        except Exception as e:
            logger.error(f"Error getting match info: {e}")
            return MatchInfo()

    def update_config(self, config: NetworkConfig) -> None:
        """Update network configuration.

        Args:
            config: New network configuration.
        """
        if config.team_number != self._config.team_number or config.robot_address != self._config.robot_address:
            self._config = config

            # Restart with new config
            if NT_AVAILABLE and self._inst:
                self.stop()
                self.start()

            logger.info(f"Network config updated: team={config.team_number}")

    def get_external_state(self) -> dict:
        """Get current external control state from NetworkTables.

        Returns:
            Dict with is_external, led_pattern, led_color (hex string), and pause_counting.
        """
        # Add a check for self.is_connected
        if not NT_AVAILABLE or not self._inst or not self.is_connected:
            return {"is_external": False, "led_pattern": 0, "led_color": "#000000", "pause_counting": False}

        try:
            is_external = self._is_external_sub.get() if self._is_external_sub else False
            led_pattern = self._led_pattern_sub.get() if self._led_pattern_sub else 0
            led_color = self._led_color_sub.get() if self._led_color_sub else "#000000"
            pause_counting = self._pause_counting_sub.get() if self._pause_counting_sub else False
            return {
                "is_external": is_external,
                "led_pattern": led_pattern,
                "led_color": led_color,
                "pause_counting": pause_counting,
            }
        except Exception as e:
            logger.error(f"Error getting external state: {e}")
            return {"is_external": False, "led_pattern": 0, "led_color": "#000000", "pause_counting": False}

    def get_reset_requested(self) -> bool:
        """Check if a reset has been requested.

        Returns True if ResetCounts is currently true.
        """
        if not NT_AVAILABLE or not self._inst or not self._reset_counts_sub:
            return False

        try:
            return self._reset_counts_sub.get()
        except Exception as e:
            logger.error(f"Error checking reset request: {e}")
            return False

    def acknowledge_reset(self) -> None:
        """Acknowledge a reset by setting ResetCounts back to false."""
        if not NT_AVAILABLE or not self._inst or not self._reset_counts_pub:
            return

        try:
            self._reset_counts_pub.set(False)
            logger.debug("Reset acknowledged (set ResetCounts to false)")
        except Exception as e:
            logger.error(f"Error acknowledging reset: {e}")

    def set_external(self, state: bool) -> None:
        """Publish external control state back to NetworkTables."""
        if NT_AVAILABLE and self._inst and self._is_external_pub:
            self._is_external_pub.set(state)

    @property
    def is_connected(self) -> bool:
        """Check if connected to NetworkTables server."""
        return self._connected

    @property
    def is_available(self) -> bool:
        """Check if NetworkTables library is available."""
        return NT_AVAILABLE
