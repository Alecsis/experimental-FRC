"""REST API routes for the hub counter."""

import asyncio
import logging
from typing import Any, Optional

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api", tags=["api"])

# These will be set by the app orchestrator
_ball_counter = None
_nt_client = None
_led_controller = None
_settings = None
_save_settings_fn = None
_app = None
_websocket_connections: list[WebSocket] = []


class CountsResponse(BaseModel):
    """Response model for ball counts."""

    channels: list[int]
    total: int


class ResetResponse(BaseModel):
    """Response model for reset operation."""

    success: bool
    counts: CountsResponse


class ConfigUpdate(BaseModel):
    """Request model for configuration updates."""

    team_number: Optional[int] = None
    robot_address: Optional[str] = None
    thresholds: Optional[dict[str, int]] = None
    colors: Optional[dict[str, list[int]]] = None


class StatusResponse(BaseModel):
    """Response model for system status."""

    counts: CountsResponse
    nt_connected: bool
    nt_available: bool
    led_active: bool
    is_external: bool = False
    match_info: Optional[dict[str, Any]] = None


def set_dependencies(
    ball_counter,
    nt_client,
    led_controller,
    settings,
    save_settings_fn,
    app=None,
):
    """Set dependencies for the API routes.

    Args:
        ball_counter: BallCounter instance.
        nt_client: NetworkTablesClient instance.
        led_controller: LEDController instance.
        settings: Current Settings instance.
        save_settings_fn: Function to save settings.
        app: HubCounterApp instance for external control.
    """
    global _ball_counter, _nt_client, _led_controller, _settings, _save_settings_fn, _app
    _ball_counter = ball_counter
    _nt_client = nt_client
    _led_controller = led_controller
    _settings = settings
    _save_settings_fn = save_settings_fn
    _app = app


@router.get("/counts", response_model=CountsResponse)
async def get_counts():
    """Get current ball counts."""
    if _ball_counter is None:
        return CountsResponse(channels=[0, 0, 0, 0], total=0)

    counts = _ball_counter.counts
    return CountsResponse(channels=counts.channels, total=counts.total)


@router.post("/counts/reset", response_model=ResetResponse)
async def reset_counts():
    """Reset all ball counts to zero."""
    if _ball_counter is None:
        return ResetResponse(
            success=False, counts=CountsResponse(channels=[0, 0, 0, 0], total=0)
        )

    counts = _ball_counter.reset()

    # Publish reset to NetworkTables
    if _nt_client:
        _nt_client.publish_counts(counts)

    # Broadcast to WebSocket clients
    await broadcast_counts(counts)

    return ResetResponse(
        success=True, counts=CountsResponse(channels=counts.channels, total=counts.total)
    )


@router.get("/config")
async def get_config():
    """Get current configuration."""
    if _settings is None:
        return {}

    return {
        "team_number": _settings.network.team_number,
        "robot_address": _settings.network.robot_address,
        "thresholds": {
            "yellow": _settings.thresholds.yellow,
            "blue": _settings.thresholds.blue,
        },
        "colors": {
            "red": list(_settings.colors.red),
            "yellow": list(_settings.colors.yellow),
            "blue": list(_settings.colors.blue),
        },
        "gpio": {
            "channel_pins": _settings.gpio.channel_pins,
            "active_low": _settings.gpio.active_low,
            "debounce_ms": _settings.gpio.debounce_ms,
        },
        "led": {
            "data_pin": _settings.led.data_pin,
            "led_count": _settings.led.led_count,
            "brightness": _settings.led.brightness,
        },
    }


@router.patch("/config")
async def update_config(config: ConfigUpdate):
    """Update configuration.

    Note: Some changes may require a restart to take effect.
    """
    global _settings

    if _settings is None:
        return {"success": False, "error": "Settings not available"}

    updated = False

    nt_needs_restart = False
    if (
        config.team_number is not None
        and config.team_number != _settings.network.team_number
    ):
        _settings.network.team_number = config.team_number
        nt_needs_restart = True

    if (
        config.robot_address is not None
        and config.robot_address != _settings.network.robot_address
    ):
        _settings.network.robot_address = config.robot_address
        nt_needs_restart = True

    if nt_needs_restart and _nt_client:
        # Since _nt_client shares the _settings.network reference,
        # it already has the new values. We just need to force the restart.
        _nt_client.stop()
        _nt_client.start()
        updated = True

    if config.thresholds is not None:
        if "yellow" in config.thresholds:
            _settings.thresholds.yellow = config.thresholds["yellow"]
        if "blue" in config.thresholds:
            _settings.thresholds.blue = config.thresholds["blue"]
        if _led_controller:
            _led_controller.update_config(threshold_config=_settings.thresholds)
        updated = True

    if config.colors is not None:
        if "red" in config.colors:
            _settings.colors.red = tuple(config.colors["red"])
        if "yellow" in config.colors:
            _settings.colors.yellow = tuple(config.colors["yellow"])
        if "blue" in config.colors:
            _settings.colors.blue = tuple(config.colors["blue"])
        if _led_controller:
            _led_controller.update_config(color_config=_settings.colors)
        updated = True

    # Save settings to file
    if updated and _save_settings_fn:
        _save_settings_fn(_settings)

    # Broadcast updated config if changed
    if updated:
        full_config = await get_config()
        await broadcast_config(full_config)

    return {"success": True, "updated": updated}


@router.get("/status", response_model=StatusResponse)
async def get_status():
    """Get full system status."""
    counts = _ball_counter.counts if _ball_counter else None
    match_info = _nt_client.get_match_info() if _nt_client else None

    return StatusResponse(
        counts=CountsResponse(
            channels=counts.channels if counts else [0, 0, 0, 0],
            total=counts.total if counts else 0,
        ),
        nt_connected=_nt_client.is_connected if _nt_client else False,
        nt_available=_nt_client.is_available if _nt_client else False,
        led_active=_led_controller.is_active if _led_controller else False,
        is_external=_app._is_external if _app else False,
        match_info=match_info.to_dict() if match_info else None,
    )


@router.post("/test/simulate-ball/{channel}")
async def simulate_ball(channel: int):
    """Simulate a ball detection on a channel (for testing)."""
    if _ball_counter is None:
        return {"success": False, "error": "Ball counter not available"}

    if not 0 <= channel < 4:
        return {"success": False, "error": "Channel must be 0-3"}

    # Check if counting is paused (only applies during external control)
    if _app and _app._is_external and _app._pause_counting:
        logger.debug(
            f"Ball simulated on channel {channel} but counting is paused — incrementing paused count"
        )
        counts = _ball_counter.increment_paused()
        if _nt_client:
            _nt_client.publish_counts(counts)
        await broadcast_counts(counts)
        return {
            "success": True,
            "channel": channel,
            "counts": CountsResponse(channels=counts.channels, total=counts.total),
        }

    counts = _ball_counter.increment(channel)

    # Publish to NetworkTables
    if _nt_client:
        _nt_client.publish_counts(counts)

    # Trigger LED pulse (skip when in external control mode)
    if _led_controller and not (_app and _app._is_external):
        loop = asyncio.get_event_loop()
        _led_controller.trigger_pulse(counts.total, loop)

    # Broadcast to WebSocket clients
    await broadcast_counts(counts)

    return {
        "success": True,
        "channel": channel,
        "counts": CountsResponse(channels=counts.channels, total=counts.total),
    }


@router.post("/external/dismiss")
async def dismiss_external():
    """Dismiss external control mode (local UI override)."""
    if _app is None:
        return {"success": False, "error": "App not available"}

    await _app.dismiss_external()
    return {"success": True}


async def broadcast_counts(counts):
    """Broadcast count update to all WebSocket clients.

    Args:
        counts: BallCounts to broadcast.
    """
    if not _websocket_connections:
        return

    message = {
        "type": "counts",
        "data": {"channels": counts.channels, "total": counts.total, "paused_total": counts.paused_total},
    }

    disconnected = []
    for websocket in _websocket_connections:
        try:
            await websocket.send_json(message)
        except Exception:
            disconnected.append(websocket)

    # Clean up disconnected clients
    for ws in disconnected:
        _websocket_connections.remove(ws)


async def broadcast_nt_status(connected: bool):
    """Broadcast NetworkTables connection status to all WebSocket clients.

    Args:
        connected: Whether NetworkTables is connected.
    """
    if not _websocket_connections:
        return

    message = {
        "type": "status",
        "data": {"nt_connected": connected},
    }

    disconnected = []
    for websocket in _websocket_connections:
        try:
            await websocket.send_json(message)
        except Exception:
            disconnected.append(websocket)

    # Clean up disconnected clients
    for ws in disconnected:
        _websocket_connections.remove(ws)


async def broadcast_external_state(is_external: bool, pattern: str, color: str, pause_counting: bool = False):
    """Broadcast external control state to all WebSocket clients.

    Args:
        is_external: Whether external control is active.
        pattern: Current pattern name.
        color: Current color hex string.
        pause_counting: Whether counting is paused.
    """
    if not _websocket_connections:
        return

    message = {
        "type": "external_state",
        "data": {
            "is_external": is_external,
            "pattern": pattern,
            "color": color,
            "pause_counting": pause_counting,
        },
    }

    disconnected = []
    for websocket in _websocket_connections:
        try:
            await websocket.send_json(message)
        except Exception:
            disconnected.append(websocket)

    for ws in disconnected:
        _websocket_connections.remove(ws)


async def broadcast_config(config: dict):
    """Broadcast config update to all WebSocket clients.

    Args:
        config: Config dict to broadcast.
    """
    if not _websocket_connections:
        return

    message = {"type": "config", "data": config}

    disconnected = []
    for websocket in _websocket_connections:
        try:
            await websocket.send_json(message)
        except Exception:
            disconnected.append(websocket)

    # Clean up disconnected clients
    for ws in disconnected:
        _websocket_connections.remove(ws)


@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time updates."""
    await websocket.accept()
    _websocket_connections.append(websocket)
    logger.info(f"WebSocket client connected, total: {len(_websocket_connections)}")

    try:
        # Send initial counts
        if _ball_counter:
            counts = _ball_counter.counts
            await websocket.send_json(
                {
                    "type": "counts",
                    "data": {"channels": counts.channels, "total": counts.total, "paused_total": counts.paused_total},
                }
            )

        # Send initial status
        await websocket.send_json(
            {
                "type": "status",
                "data": {"nt_connected": _nt_client.is_connected if _nt_client else False},
            }
        )

        # Send initial external state
        pattern_name = ""
        color_hex = ""
        if _app and _app._is_external:
            # Replicate the name mapping from app.py
            PATTERN_NAMES = {0: "Solid", 1: "Blink", 2: "Racing"}
            pattern_name = PATTERN_NAMES.get(_app._last_external_pattern, "Unknown")
            color_hex = _app._last_external_color

        await websocket.send_json(
            {
                "type": "external_state",
                "data": {
                    "is_external": _app._is_external if _app else False,
                    "pattern": pattern_name,
                    "color": color_hex,
                    "pause_counting": _app._pause_counting if _app else False,
                },
            }
        )

        # Keep connection alive and handle messages
        while True:
            data = await websocket.receive_text()
            # Handle ping/pong for keepalive
            if data == "ping":
                await websocket.send_text("pong")

    except WebSocketDisconnect:
        _websocket_connections.remove(websocket)
        logger.info(
            f"WebSocket client disconnected, total: {len(_websocket_connections)}"
        )
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        if websocket in _websocket_connections:
            _websocket_connections.remove(websocket)
