#!/usr/bin/env python3
"""Simple CLI tool to control Hub Counter via NetworkTables (robot simulator)."""

import argparse
import logging
import sys
import time

try:
    import ntcore
except ImportError:
    print("Error: ntcore not available. Install with: pip install robotpy-ntcore")
    sys.exit(1)

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)


class NTController:
    """NetworkTables controller for Hub Counter."""

    def __init__(self, team_number: int = 6328, server: str = None):
        """Initialize NT controller.

        Args:
            team_number: FRC team number for server discovery.
            server: Explicit server address (e.g., "localhost" or "10.63.28.2").
        """
        self.inst = ntcore.NetworkTableInstance.getDefault()

        if server:
            logger.info(f"Connecting to server: {server}")
            self.inst.setServer(server)
        else:
            logger.info(f"Using team number for server discovery: {team_number}")
            self.inst.setServerTeam(team_number)

        self.inst.startClient4("NTController")

        # Get HubCounter table
        hub_table = self.inst.getTable("HubCounter")

        # Create publishers
        self.is_external_pub = hub_table.getBooleanTopic("IsExternal").publish()
        self.led_pattern_pub = hub_table.getIntegerTopic("Led/Pattern").publish()
        self.led_color_pub = hub_table.getStringTopic("Led/Color").publish()
        self.reset_counts_pub = hub_table.getBooleanTopic("ResetCounts").publish()
        self.pause_counting_pub = hub_table.getBooleanTopic("PauseCounting").publish()

        # Create subscribers to read current values
        self.total_count_sub = hub_table.getIntegerTopic("TotalCount").subscribe(0)
        self.channel_counts_sub = hub_table.getIntegerArrayTopic("ChannelCounts").subscribe([])
        self.reset_counts_sub = hub_table.getBooleanTopic("ResetCounts").subscribe(False)

        # Wait for connection
        logger.info("Waiting for NetworkTables connection...")
        for i in range(50):  # 5 second timeout
            if self.inst.isConnected():
                logger.info("Connected to NetworkTables!")
                break
            time.sleep(0.1)
        else:
            logger.warning("Not connected to NetworkTables (timeout)")

    def set_external(self, enabled: bool):
        """Enable/disable external control mode."""
        self.is_external_pub.set(enabled)
        logger.info(f"IsExternal set to: {enabled}")

    def set_pause_counting(self, paused: bool):
        """Enable/disable pause counting (prevents score incrementing when true)."""
        self.pause_counting_pub.set(paused)
        logger.info(f"PauseCounting set to: {paused}")

    def set_pattern(self, pattern: int):
        """Set LED pattern.

        Args:
            pattern: 0=Solid, 1=Blink, 2=Racing
        """
        pattern_names = {0: "Solid", 1: "Blink", 2: "Racing"}
        self.led_pattern_pub.set(pattern)
        logger.info(f"Led/Pattern set to: {pattern} ({pattern_names.get(pattern, 'Unknown')})")

    def set_color(self, r: int, g: int, b: int):
        """Set LED color as RGB.

        Args:
            r: Red (0-255)
            g: Green (0-255)
            b: Blue (0-255)
        """
        hex_color = f"#{r:02x}{g:02x}{b:02x}"
        self.led_color_pub.set(hex_color)
        logger.info(f"Led/Color set to: {hex_color} (RGB={r},{g},{b})")

    def reset_counts(self):
        """Request count reset by setting ResetCounts to true.

        The hub counter will perform the reset and acknowledge by setting it back to false.
        """
        self.reset_counts_pub.set(True)
        logger.info("ResetCounts requested (set to true, waiting for hub to acknowledge)")

        # Wait a moment and check if hub acknowledged
        time.sleep(0.2)
        if self.inst.isConnected():
            # Read the value back to see if hub acknowledged
            current = self.reset_counts_sub.get() if hasattr(self, 'reset_counts_sub') else None
            if current is False:
                logger.info("Reset acknowledged by hub counter")
            else:
                logger.info("Waiting for hub counter to acknowledge...")

    def get_counts(self):
        """Get current ball counts."""
        total = self.total_count_sub.get()
        channels = self.channel_counts_sub.get()
        logger.info(f"Current counts - Total: {total}, Channels: {list(channels)}")
        return total, channels

    def stop(self):
        """Stop the controller."""
        self.is_external_pub.close()
        self.led_pattern_pub.close()
        self.led_color_pub.close()
        self.reset_counts_pub.close()
        self.pause_counting_pub.close()
        self.total_count_sub.close()
        self.channel_counts_sub.close()
        self.reset_counts_sub.close()
        self.inst.stopClient()
        logger.info("Stopped")


def fancy_demo(controller: NTController):
    """Run a fancy 30-second demo showcasing patterns and colors."""
    print("\n=== Starting Fancy Demo (30 seconds) ===\n")

    # Enable external control
    controller.set_external(True)
    time.sleep(1)

    # Demo sequence
    demos = [
        # Solid colors showcase (8 seconds)
        ("Solid Red", 0, 255, 0, 0, 1.5),
        ("Solid Green", 0, 0, 255, 0, 1.5),
        ("Solid Blue", 0, 0, 0, 255, 1.5),
        ("Solid Yellow", 0, 255, 255, 0, 1.5),
        ("Solid Purple", 0, 255, 0, 255, 1.5),
        ("Solid Cyan", 0, 0, 255, 255, 1.5),

        # Blink patterns (8 seconds)
        ("Blink Orange", 1, 255, 128, 0, 2),
        ("Blink Magenta", 1, 255, 0, 128, 2),
        ("Blink Lime", 1, 128, 255, 0, 2),
        ("Blink Hot Pink", 1, 255, 20, 147, 2),

        # Racing patterns (12 seconds)
        ("Racing Red", 2, 255, 0, 0, 3),
        ("Racing Green", 2, 0, 255, 0, 3),
        ("Racing Blue", 2, 0, 0, 255, 3),
        ("Racing Rainbow (Purple)", 2, 128, 0, 255, 3),

        # Grand finale - rapid color changes with solid (2 seconds)
        ("Finale White", 0, 255, 255, 255, 0.5),
        ("Finale Gold", 0, 255, 215, 0, 0.5),
        ("Finale Aqua", 0, 0, 255, 255, 0.5),
        ("Finale Off", 0, 0, 0, 0, 0.5),
    ]

    total_time = 0
    for name, pattern, r, g, b, duration in demos:
        print(f"⚡ {name:25} (Pattern: {pattern}, RGB: {r:3},{g:3},{b:3})")
        controller.set_pattern(pattern)
        controller.set_color(r, g, b)
        time.sleep(duration)
        total_time += duration

    # Disable external control
    print("\n✨ Demo complete!")
    controller.set_external(False)
    print(f"Total demo time: {total_time:.1f} seconds\n")


def interactive_mode(controller: NTController):
    """Run interactive CLI mode."""
    print("\n=== Hub Counter NetworkTables Controller ===")
    print("Commands:")
    print("  external on|off       - Enable/disable external control")
    print("  pause on|off          - Enable/disable pause counting")
    print("  pattern <0-2>         - Set pattern (0=Solid, 1=Blink, 2=Racing)")
    print("  color <r> <g> <b>     - Set RGB color (0-255 each)")
    print("  reset                 - Reset ball counts")
    print("  counts                - Show current counts")
    print("  quit                  - Exit")
    print()

    try:
        while True:
            try:
                cmd = input("> ").strip().lower().split()
                if not cmd:
                    continue

                if cmd[0] == "quit" or cmd[0] == "exit":
                    break

                elif cmd[0] == "external":
                    if len(cmd) != 2:
                        print("Usage: external on|off")
                        continue
                    enabled = cmd[1] in ("on", "true", "1")
                    controller.set_external(enabled)

                elif cmd[0] == "pause":
                    if len(cmd) != 2:
                        print("Usage: pause on|off")
                        continue
                    paused = cmd[1] in ("on", "true", "1")
                    controller.set_pause_counting(paused)

                elif cmd[0] == "pattern":
                    if len(cmd) != 2:
                        print("Usage: pattern <0-2>")
                        continue
                    try:
                        pattern = int(cmd[1])
                        if pattern not in (0, 1, 2):
                            print("Pattern must be 0 (Solid), 1 (Blink), or 2 (Racing)")
                            continue
                        controller.set_pattern(pattern)
                    except ValueError:
                        print("Pattern must be a number")

                elif cmd[0] == "color":
                    if len(cmd) != 4:
                        print("Usage: color <r> <g> <b>")
                        continue
                    try:
                        r, g, b = int(cmd[1]), int(cmd[2]), int(cmd[3])
                        if not all(0 <= x <= 255 for x in (r, g, b)):
                            print("RGB values must be 0-255")
                            continue
                        controller.set_color(r, g, b)
                    except ValueError:
                        print("RGB values must be numbers")

                elif cmd[0] == "reset":
                    controller.reset_counts()

                elif cmd[0] == "counts":
                    controller.get_counts()

                else:
                    print(f"Unknown command: {cmd[0]}")

            except KeyboardInterrupt:
                print()
                break
            except Exception as e:
                print(f"Error: {e}")

    finally:
        controller.stop()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Control Hub Counter via NetworkTables (robot simulator)"
    )
    parser.add_argument(
        "--team",
        type=int,
        default=6328,
        help="FRC team number (default: 6328)"
    )
    parser.add_argument(
        "--server",
        type=str,
        help="Explicit server address (e.g., 'localhost' or '10.63.28.2')"
    )

    # Command-line options for one-shot commands
    parser.add_argument("--external", choices=["on", "off"], help="Set external control")
    parser.add_argument("--pause", choices=["on", "off"], help="Set pause counting")
    parser.add_argument("--pattern", type=int, choices=[0, 1, 2], help="Set LED pattern")
    parser.add_argument("--color", nargs=3, type=int, metavar=("R", "G", "B"), help="Set RGB color")
    parser.add_argument("--reset", action="store_true", help="Reset ball counts")
    parser.add_argument("--counts", action="store_true", help="Show current counts")
    parser.add_argument("--fancy-demo", action="store_true", help="Run a 30-second demo showcasing patterns and colors")

    args = parser.parse_args()

    controller = NTController(team_number=args.team, server=args.server)

    # Check for fancy demo mode
    if args.fancy_demo:
        fancy_demo(controller)
        controller.stop()
        return

    # Check if any one-shot commands were specified
    one_shot = any([
        args.external,
        args.pause,
        args.pattern is not None,
        args.color,
        args.reset,
        args.counts
    ])

    if one_shot:
        # Execute one-shot commands
        if args.external:
            controller.set_external(args.external == "on")
        if args.pause:
            controller.set_pause_counting(args.pause == "on")
        if args.pattern is not None:
            controller.set_pattern(args.pattern)
        if args.color:
            controller.set_color(*args.color)
        if args.reset:
            controller.reset_counts()
        if args.counts:
            controller.get_counts()
        controller.stop()
    else:
        # Run interactive mode
        interactive_mode(controller)


if __name__ == "__main__":
    main()
