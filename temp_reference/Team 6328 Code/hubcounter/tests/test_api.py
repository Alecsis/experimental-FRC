"""Tests for the REST API."""

import pytest
from fastapi.testclient import TestClient

from hub_counter.config.settings import Settings
from hub_counter.gpio.ball_counter import BallCounter
from hub_counter.web.routes.api import set_dependencies
from hub_counter.web.server import create_app


@pytest.fixture
def client():
    """Create a test client with mock dependencies."""
    app = create_app()

    # Set up mock dependencies
    ball_counter = BallCounter()
    settings = Settings()

    set_dependencies(
        ball_counter=ball_counter,
        nt_client=None,
        led_controller=None,
        settings=settings,
        save_settings_fn=lambda s: None,
    )

    with TestClient(app) as client:
        yield client, ball_counter


def test_get_counts(client):
    """Test getting current counts."""
    test_client, _ = client
    response = test_client.get("/api/counts")

    assert response.status_code == 200
    data = response.json()
    assert "channels" in data
    assert "total" in data
    assert data["channels"] == [0, 0, 0, 0]
    assert data["total"] == 0


def test_reset_counts(client):
    """Test resetting counts."""
    test_client, ball_counter = client

    # Add some counts first
    ball_counter.increment(0)
    ball_counter.increment(1)

    response = test_client.post("/api/counts/reset")

    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True
    assert data["counts"]["total"] == 0


def test_get_config(client):
    """Test getting configuration."""
    test_client, _ = client
    response = test_client.get("/api/config")

    assert response.status_code == 200
    data = response.json()
    assert "team_number" in data
    assert "thresholds" in data
    assert "colors" in data


def test_update_config(client):
    """Test updating configuration."""
    test_client, _ = client

    response = test_client.patch(
        "/api/config",
        json={
            "team_number": 1234,
            "thresholds": {"yellow": 50, "blue": 200},
        },
    )

    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True


def test_get_status(client):
    """Test getting system status."""
    test_client, _ = client
    response = test_client.get("/api/status")

    assert response.status_code == 200
    data = response.json()
    assert "counts" in data
    assert "nt_connected" in data
    assert "led_active" in data


def test_simulate_ball(client):
    """Test simulating a ball detection."""
    test_client, _ = client

    response = test_client.post("/api/test/simulate-ball/0")

    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True
    assert data["channel"] == 0
    assert data["counts"]["channels"][0] == 1
    assert data["counts"]["total"] == 1


def test_simulate_ball_invalid_channel(client):
    """Test simulating ball on invalid channel."""
    test_client, _ = client

    response = test_client.post("/api/test/simulate-ball/5")

    assert response.status_code == 200
    data = response.json()
    assert data["success"] is False
