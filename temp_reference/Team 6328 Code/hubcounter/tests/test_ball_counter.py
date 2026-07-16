"""Tests for the BallCounter class."""

import pytest

from hub_counter.gpio.ball_counter import BallCounter


def test_initial_counts():
    """Test that initial counts are all zero."""
    counter = BallCounter()
    counts = counter.counts

    assert counts.channels == [0, 0, 0, 0]
    assert counts.total == 0


def test_increment_channel():
    """Test incrementing a single channel."""
    counter = BallCounter()

    counts = counter.increment(0)
    assert counts.channels[0] == 1
    assert counts.total == 1

    counts = counter.increment(0)
    assert counts.channels[0] == 2
    assert counts.total == 2


def test_increment_multiple_channels():
    """Test incrementing different channels."""
    counter = BallCounter()

    counter.increment(0)
    counter.increment(1)
    counter.increment(2)
    counts = counter.increment(3)

    assert counts.channels == [1, 1, 1, 1]
    assert counts.total == 4


def test_increment_invalid_channel():
    """Test that invalid channel raises ValueError."""
    counter = BallCounter()

    with pytest.raises(ValueError):
        counter.increment(-1)

    with pytest.raises(ValueError):
        counter.increment(4)


def test_reset():
    """Test resetting counts."""
    counter = BallCounter()

    counter.increment(0)
    counter.increment(1)
    counter.increment(2)

    counts = counter.reset()

    assert counts.channels == [0, 0, 0, 0]
    assert counts.total == 0


def test_callback():
    """Test count change callback."""
    counter = BallCounter()
    callback_counts = []

    def on_change(counts):
        callback_counts.append(counts)

    counter.on_count_change(on_change)

    counter.increment(0)
    counter.increment(1)

    assert len(callback_counts) == 2
    assert callback_counts[0].total == 1
    assert callback_counts[1].total == 2


def test_remove_callback():
    """Test removing a callback."""
    counter = BallCounter()
    callback_counts = []

    def on_change(counts):
        callback_counts.append(counts)

    counter.on_count_change(on_change)
    counter.increment(0)

    counter.remove_callback(on_change)
    counter.increment(1)

    assert len(callback_counts) == 1


def test_set_counts():
    """Test setting counts directly."""
    counter = BallCounter()

    counts = counter.set_counts([10, 20, 30, 40])

    assert counts.channels == [10, 20, 30, 40]
    assert counts.total == 100
