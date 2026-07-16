# Hub Counter

Ball counter for FRC Team 6328's hub return system. Monitors four channels via GPIO, displays feedback on WS2813 LED strips, publishes counts over NetworkTables, and serves a live web dashboard.

## Requirements

- Python 3.11+
- Raspberry Pi (for GPIO and LED hardware; mock mode available for development)

## Setup

```bash
cd hub_counter

# Create a virtual environment
python3 -m venv venv
source venv/bin/activate

# Install core dependencies
pip install -r requirements.txt

# On a Raspberry Pi, also install hardware dependencies
pip install gpiozero rpi-ws281x lgpio
```

## Running

### On a Raspberry Pi

```bash
# Or directly
python main.py

# Or as a module
python -m hub_counter
```

Note that this code must run as root to access the hardware DMA channels needed to interact with the LED drivers (and to bind to port 80).

### Development (no Pi hardware)

```bash
MOCK_HARDWARE=1 python main.py
```

Mock mode simulates GPIO inputs and LED output so you can develop and test on any machine.

### Options

```
  -c, --config PATH    Path to config file (default: config.yaml)
  -v, --verbose        Enable debug logging
      --version        Show version and exit
```

## Configuration

Edit `config.yaml` to change runtime settings:

```yaml
gpio:
  channel_pins: [23, 24, 25, 16]   # GPIO pins for the 4 channels
  active_low: false                  # Pin reads LOW when a ball is not detected
  debounce_ms: 10                    # Debounce window in milliseconds

led:
  data_pin: 21        # GPIO data pin for WS2813 strips
  led_count: 12        # LEDs per strip (2 strips, 12 total)
  brightness: 200     # 0-255

network:
  team_number: 6328
  robot_address: 10.63.28.2

thresholds:
  yellow: 100          # Total fuel to switch LED pulse to yellow
  blue: 360            # Total fuel to switch LED pulse to blue

colors:
  red: [255, 0, 0]
  yellow: [255, 200, 0]
  blue: [0, 100, 255]

web:
  host: 0.0.0.0
  port: 80
```

All of these settings can also be adjusted at runtime through the web dashboard's config screen.

## Web Dashboard

Once running, open `http://<pi-ip>` in a browser. The dashboard shows:

- Live aggregate and per-channel ball counts
- Simulate buttons for testing without hardware
- Reset button to zero out counts
- Configuration panel for network, thresholds, and LED colors

Real-time updates are delivered over WebSocket.

## NetworkTables Robot Simulator

The `nt_control.py` script simulates robot control over NetworkTables. Use it to test external LED control without a real robot.

### Interactive Mode

```bash
# Connect to local hub counter
./nt_control.py --server localhost

# Available commands:
> external on          # Enable external control mode
> pattern 1            # Set pattern (0=Solid, 1=Blink, 2=Racing)
> color 255 0 0        # Set RGB color (red)
> reset                # Reset ball counts
> counts               # Show current counts
> quit                 # Exit
```

### One-Shot Commands

```bash
# Enable external control with red blinking LEDs
./nt_control.py --server localhost --external on --pattern 1 --color 255 0 0

# Reset counts via NetworkTables
./nt_control.py --server localhost --reset

# Check current counts
./nt_control.py --server localhost --counts
```

### Example: Full External Control Test

```bash
# Terminal 1: Start hub counter in mock mode
MOCK_HARDWARE=1 python main.py

# Terminal 2: Simulate robot control
./nt_control.py --server localhost
> external on
> pattern 2            # Racing pattern
> color 0 255 0        # Green
```

The hub counter will now display a green racing pattern controlled by NetworkTables, just like it would when connected to a real robot.

### Fancy Demo Mode

Run an automated 30-second demonstration showcasing all patterns and colors:

```bash
./nt_control.py --server localhost --fancy-demo
```

The demo automatically cycles through solid colors, blinking patterns, and racing effects with various colors.

## API

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/counts` | Current ball counts |
| POST | `/api/counts/reset` | Reset all counts to zero |
| GET | `/api/config` | Current configuration |
| PATCH | `/api/config` | Update configuration |
| GET | `/api/status` | System status |
| POST | `/api/test/simulate-ball/{channel}` | Simulate a ball on channel 0-3 |
| WS | `/api/ws` | WebSocket for live count updates |

## Tests

```bash
pip install -e ".[dev]"
pytest
```

## License

MIT
