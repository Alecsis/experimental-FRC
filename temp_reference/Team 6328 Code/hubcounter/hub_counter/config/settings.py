"""Configuration settings using Pydantic."""

from pathlib import Path
from typing import Optional

import yaml
from pydantic import BaseModel, Field


class GPIOConfig(BaseModel):
    """GPIO configuration."""

    channel_pins: list[int] = Field(default=[23, 24, 25, 16], min_length=4, max_length=4)
    active_low: bool = True
    debounce_ms: int = Field(default=10, ge=10, le=500)


class LEDConfig(BaseModel):
    """LED strip configuration."""

    data_pin: int = 21
    led_count: int = Field(default=12, ge=1, le=100)
    brightness: int = Field(default=200, ge=0, le=255)


class ThresholdConfig(BaseModel):
    """Ball count thresholds for color changes."""

    yellow: int = Field(default=100, ge=1)
    blue: int = Field(default=360, ge=1)


class ColorConfig(BaseModel):
    """RGB color configuration."""

    red: tuple[int, int, int] = (255, 0, 0)
    yellow: tuple[int, int, int] = (255, 200, 0)
    blue: tuple[int, int, int] = (0, 100, 255)


class NetworkConfig(BaseModel):
    """NetworkTables configuration."""

    team_number: int = Field(default=6328, ge=1, le=9999)
    robot_address: str = "10.63.28.2"


class WebConfig(BaseModel):
    """Web server configuration."""

    host: str = "0.0.0.0"
    port: int = Field(default=80, ge=1, le=65535)


class Settings(BaseModel):
    """Main application settings."""

    gpio: GPIOConfig = Field(default_factory=GPIOConfig)
    led: LEDConfig = Field(default_factory=LEDConfig)
    thresholds: ThresholdConfig = Field(default_factory=ThresholdConfig)
    colors: ColorConfig = Field(default_factory=ColorConfig)
    network: NetworkConfig = Field(default_factory=NetworkConfig)
    web: WebConfig = Field(default_factory=WebConfig)


def load_settings(config_path: Optional[Path] = None) -> Settings:
    """Load settings from YAML config file.

    Args:
        config_path: Path to config file. Defaults to config.yaml in current directory.

    Returns:
        Settings instance with loaded configuration.
    """
    if config_path is None:
        config_path = Path("config.yaml")

    if config_path.exists():
        with open(config_path) as f:
            config_data = yaml.safe_load(f) or {}
        return Settings(**config_data)

    return Settings()


def save_settings(settings: Settings, config_path: Optional[Path] = None) -> None:
    """Save settings to YAML config file.

    Args:
        settings: Settings instance to save.
        config_path: Path to config file. Defaults to config.yaml in current directory.
    """
    if config_path is None:
        config_path = Path("config.yaml")

    config_data = settings.model_dump()
    with open(config_path, "w") as f:
        yaml.safe_dump(config_data, f, default_flow_style=False)
