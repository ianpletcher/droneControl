# Ground Station Setup & Run Guide

This guide explains how to set up the Python 3.12 virtual environment required to run the ground station script (`ground_control/main.py`) on macOS or Linux. GStreamer's Python bindings require Python 3.12 specifically — the system Python (3.14+) will not work because the `gi` (PyGObject) module is only available for properly configured Python versions.

---

## Prerequisites

**macOS:** [Homebrew](https://brew.sh) installed
**Linux:** `apt` (Debian/Ubuntu) installed

---

## Step 1 — Install GStreamer and Python 3.12

**macOS (Homebrew):**
```bash
brew install python@3.12
brew install gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly gst-libav
brew install pygobject3
```

**Linux (Ubuntu/Debian):**
```bash
sudo apt update
sudo apt install python3.12 python3.12-venv python3.12-dev
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav
sudo apt install libgirepository1.0-dev gobject-introspection python3-gi
```

This may take several minutes. Once complete, verify Python 3.12 is available:

```bash
python3.12 --version
# Expected output: Python 3.12.x
```



---

## Step 2 — Install the `gi` module for Python 3.12

The `gi` module bridges Python and GStreamer.

**macOS (already installed via Homebrew above):** Skip this step

**Linux:** Install using pip with the `--break-system-packages` flag:
```bash
python3.12 -m pip install --break-system-packages pygobject
```

---

## Step 3 — Create the virtual environment

The venv **must** be created with `--system-site-packages` so it can see the `gi` module. A venv created without this flag will not be able to import GStreamer.

```bash
python3.12 -m venv ~/drone-gs-venv --system-site-packages
```

---

## Step 4 — Activate the venv and install remaining dependencies

```bash
source ~/drone-gs-venv/bin/activate
pip install pygame numpy msgpack
```

Verify the critical imports work before proceeding:

```bash
python3 -c "import gi; gi.require_version('Gst', '1.0'); from gi.repository import Gst; print('GStreamer OK')"
python3 -c "import pygame; print('pygame OK')"
python3 -c "import msgpack; print('msgpack OK')"
```

All three should print their respective OK messages with no errors. If `GStreamer OK` fails, revisit Steps 1 and 2.

---

## Step 5 — Connect to the Radxa ground station

Before running the ground station script, the Radxa Zero 3W must be connected to your workstation via USB-C in Ethernet Gadget mode.

1. Plug the Radxa into your workstation via USB-C
2. Verify the connection — a new network interface should appear. Confirm with:

```bash
ping 10.55.0.1
```

A successful ping confirms the USB gadget link is up. If it times out, check that the Radxa has booted fully and that the `usb0` interface is configured on the Radxa side.

To SSH into the Radxa directly from your laptop:

```bash
ssh rock@10.55.0.1
```

To SSH into the Pi 5 via the Radxa (double-hop):

```bash
ssh -J rock@10.55.0.1 pi@10.5.0.2
```

---

## Step 6 — Run the ground station

With the venv active and the Radxa connected:

```bash
source ~/drone-gs-venv/bin/activate
python3 ground_control/main.py
```

The Pygame window will open and display a waiting message until video begins arriving from the drone. Once the drone-side script (`air_pi.py`) is running, the live video feed and detection overlays will appear.

To stop the ground station, press `Ctrl+C` in the terminal or close the Pygame window.

---

## Network Reference

| Device | Interface | IP Address |
|---|---|---|
| Raspberry Pi 5 (drone) | drone-wfb | 10.5.0.2 |
| Radxa Zero 3W (bridge) | drone-wfb | 10.5.0.1 |
| Radxa Zero 3W (USB gadget) | usb0 | 10.55.0.1 |
| Laptop (workstation) | USB gadget | 10.55.0.2 |

| Stream | Protocol | Port |
|---|---|---|
| Video (H.264/RTP) | UDP | 5602 |
| Tracking data (msgpack) | UDP | 5601 |
| Commands (JSON) | TCP | 5603 |

---

## Troubleshooting

**`ModuleNotFoundError: No module named 'gi'`**
The venv was created without `--system-site-packages`. Delete it and recreate using the command in Step 3.

**`GStreamer ERROR` on startup**
Confirm all GStreamer plugins were installed in Step 1. Run `gst-inspect-1.0 x264enc` to verify the encoder plugin is present.

**Pygame window opens but no video appears**
Confirm the Radxa forwarder (`drone_forwarder.py`) is running on the Radxa and that `ping 10.55.0.1` succeeds from your laptop. Also confirm `air_pi.py` is running on the Pi.

**`ping 10.55.0.1` times out**
The USB gadget interface is not up. Try unplugging and replugging the USB-C cable. If the issue persists, SSH into the Radxa via another method and verify the `usb0` interface has its static IP assigned.
