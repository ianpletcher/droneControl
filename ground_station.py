import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib
# do we not need to specify gst_bin as gst?
import os
import sys
import pygame
import numpy as np  # <-- ADDED THIS MISSING IMPORT
import threading
import socket
import json
import msgpack
import struct
import time
import tomllib

#Ideas for ground station: 
# - Instead of just having users click on bbox to track, have a "selection mode" or a key on side of display. Make it hidden at first and popout with an arrow

''' 
Make sure camera stream is split 640*640 for AI processing and 1280*720 for display.
Split the camera feed with a tee in GStreamer.
Implement Multiplexing on the 2.4ghz 
Use 5.0 as primary until range exceeds 5.0 capabilities, then switch to 2.4ghz
Send Commands over TCP and Video over UDP.
'''

# -----------------------------------------------------------------------------------------------
# Network Configuration (configurable via config.toml)
# -----------------------------------------------------------------------------------------------
# Original hard-coded values (kept commented so you can revert if needed):
# NOTE: This Pi's IP is 192.168.0.169.
# We are listening on all interfaces (0.0.0.0).
# AIR_PI_IP = "10.5.0.2"      # IP of the Air Pi (for sending commands to)
# VIDEO_STREAM_PORT = 5602    # Port to listen on for H.264 video
# DATA_PORT = 5601            # Port to listen on for tracking data
# COMMAND_PORT = 5603         # Port on the Air Pi to send commands to

# Load configuration from `config.toml` (repo root) if available.
def load_config(path=None):
    path = path or os.environ.get("DRONE_CONFIG", "config.toml")
    if not os.path.exists(path):
        return {}
    try:
        import tomllib as _toml  # Python 3.11+
        with open(path, "rb") as f:
            return _toml.load(f)
    except Exception:
        try:
            import tomllib as _toml  # toml parser
            with open(path, "rb") as f:
                return _toml.load(f)
        except Exception:
            return {}


# Read config and apply defaults
_CFG = load_config()
_NET = _CFG.get("network", {})
AIR_PI_IP = _NET.get("air_ip", "10.5.0.2")
VIDEO_STREAM_PORT = int(_NET.get("video_port", 5602))
DATA_PORT = int(_NET.get("data_port", 5601))
COMMAND_PORT = int(_NET.get("command_port", 5603))

# Must match the constants in air_pi.py exactly.
PACKET_HEADER_FMT = "!I H H"   # seq (uint32), frag_idx (uint16), frag_count (uint16)
PACKET_HEADER_LEN = struct.calcsize(PACKET_HEADER_FMT)   # 8 byte header 

# Drop any reassembly buffer that has not completed within this time (seconds).
# Default tuned for low-latency telemetry; can be overridden in config.toml
_TEL = _CFG.get("telemetry", {})
REASSEMBLY_TIMEOUT = float(_TEL.get("reassembly_timeout", 0.3))

# -----------------------------------------------------------------------------------------------
# Application State
# -----------------------------------------------------------------------------------------------
class AppState:
    """Manages shared state between GStreamer and UI threads"""
   
    def __init__(self):
        self.pygame_frame = None                        # The most recent numpy frame
        self.tracking_data = []                         # The most recent tracking data
        self.last_click_pos = None                      # (x, y) tuple of the last click
        self.current_target_id = None                   # The ID we are currently tracking
        self.gst_error_event = threading.Event()        # Set if GStreamer fails
        self.data_thread_stop_event = threading.Event() # Set to signal data thread to stop
       
        # Thread locks for synchronization
        self.frame_lock = threading.Lock()
        self.tracking_data_lock = threading.Lock()
        self.click_lock = threading.Lock()
        self.target_lock = threading.Lock()

# -----------------------------------------------------------------------------------------------
# GStreamer Video Receiver Thread
# -----------------------------------------------------------------------------------------------
def run_gstreamer_receiver(app_state, main_loop):
    """Run GStreamer in separate thread to receive and display video"""
   
    # HEVC is more efficent, but I do not know technical specifics
    # Could be that it's difficult to implement or more computationally intensive on one end. 

    # This pipeline receives the H.264 stream, decodes it, and sends it to our app
    # FIXED: Added proper jitter buffer, timestamps, and latency management
    pipeline_str = ( 
        f"udpsrc port={VIDEO_STREAM_PORT} caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96\" ! "
        "rtpjitterbuffer latency=50 drop-on-latency=true ! "  # Low latency jitter buffer
        "rtph264depay ! "
        "h264parse ! "
        "avdec_h264 ! "
        "videoscale ! video/x-raw,width=1280,height=720 ! "  # Match sender resolution
        "videoconvert ! "
        "video/x-raw,format=RGB ! "
        "appsink name=appsink emit-signals=true sync=false max-buffers=1 drop=true"  # Drop old frames aggressively
    )
   
    """  Notes for pipeline tuning and testing:
   
        max-buffers=1 + drop=true ensures your UI always shows the latest frame but sacrifices smoothness; 
        increase max-buffers or remove drop=true for smoother playback if latency budget allows.
        The caps must match the sender's RTP parameters; mismatches (payload type, resolution, framerate) 
        can break decoding or require extra negotiation.
        If debugging stream issues, temporarily raise rtpjitterbuffer latency (e.g., 100-200 ms) 
        to see if artifacts are jitter-related.
        
    """

#need to make pipeine string robust and variable if network conditions change.

    pipeline = None
    try:
        pipeline = Gst.parse_launch(pipeline_str)
       
        # Find the appsink
        appsink = pipeline.get_by_name("appsink")
        if not appsink:
            print("ERROR: Could not find 'appsink' element in pipeline.")
            raise RuntimeError("Missing appsink element")
           
        # Connect our callback
        # appsink.set_property('emit-signals', True) this is already set in pipe
        appsink.connect('new-sample', on_new_video_sample, app_state)
       
        # Set pipeline to PLAYING
        pipeline.set_state(Gst.State.PLAYING)
        print("GStreamer receiver pipeline is running...")
       
        # Run the GLib main loop
        main_loop.run()
       
    except Exception as e:
        print(f"GStreamer error: {e}")
    finally:
        print("GStreamer thread stopping...")
        if pipeline:
            pipeline.set_state(Gst.State.NULL)
        app_state.gst_error_event.set()


def on_new_video_sample(appsink, app_state):
    """Callback for the appsink 'new-sample' signal (from GStreamer thread)"""
   
    sample = appsink.emit('pull-sample')
    if not sample:
        return Gst.FlowReturn.OK
       
    buffer = sample.get_buffer()
    if not buffer:
        return Gst.FlowReturn.OK
   
    # Get frame size and format from caps
    caps = sample.get_caps()
    structure = caps.get_structure(0)
    width = structure.get_value('width')
    height = structure.get_value('height')
    # format = structure.get_string('format') # Should be RGB
   
    # Map buffer to readable numpy array

    #buffer.map needs t obe initalized as none before the try otherwise we can get two exceptions thrown 
    try:
        (result, map_info) = buffer.map(Gst.MapFlags.READ)
        if result:
            # Create numpy array from buffer
            numpy_frame = np.ndarray(
                (height, width, 3),
                buffer=map_info.data,
                dtype=np.uint8
            )
           
            # Update frame for pygame (thread-safe)
            with app_state.frame_lock:
                # We must copy() the frame, otherwise it becomes invalid
                # when we unmap the buffer
                app_state.pygame_frame = numpy_frame.copy()
               
    except Exception as e:
        print(f"Error processing video frame: {e}")
    finally:
        buffer.unmap(map_info)
       
    return Gst.FlowReturn.OK

# -----------------------------------------------------------------------------------------------
# Tracking Data Receiver Thread
# -----------------------------------------------------------------------------------------------

# need to modify for msgpack

def run_data_receiver(app_state):
    """
    Listens for fragmented msgpack telemetry packets from the Air Pi.

    Packet header (8 bytes, big-endian):
        seq        (uint32) message sequence number, increments every frame
        frag_idx   (uint16) 0-based index of this fragment within the message
        frag_count (uint16) total fragments that make up this message

    Packet-loss behaviour:
        - If some fragments for a message never arrive, the message is silently
          dropped after REASSEMBLY_TIMEOUT seconds.  The display keeps showing
          the last fully-received set of bounding boxes no partial/corrupt
          frames are ever rendered.
        - Out-of-order or duplicate datagrams for already-processed messages
          are discarded using the seq number.
    """
    # {seq: {'frags': {frag_idx: bytes}, 'total': int, 'first_seen': float}}
    reassembly = {}
    last_processed_seq = -1

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', DATA_PORT))
        s.settimeout(1.0)   # allows the stop-event check on every timeout
        print(f"Data receiver listening on port {DATA_PORT}")

        while not app_state.data_thread_stop_event.is_set():
            try:
                data, addr = s.recvfrom(65536)

                # ── Parse header ──────────────────────────────────────────────
                if len(data) < PACKET_HEADER_LEN:
                    continue   # runt packet, ignore

                seq, frag_idx, frag_count = struct.unpack(
                    PACKET_HEADER_FMT, data[:PACKET_HEADER_LEN]
                )
                payload = data[PACKET_HEADER_LEN:]

                # ── Discard stale / duplicate datagrams ───────────────────────
                if seq <= last_processed_seq:
                    continue

                # ── Buffer this fragment ──────────────────────────────────────
                if seq not in reassembly:
                    reassembly[seq] = {
                        'frags':      {},
                        'total':      frag_count,
                        'first_seen': time.time(),
                    }
                reassembly[seq]['frags'][frag_idx] = payload

                # ── Try to assemble once all fragments have arrived ───────────
                entry = reassembly[seq]
                if len(entry['frags']) == entry['total']:
                    body = b''.join(
                        entry['frags'][i] for i in range(entry['total'])
                    )
                    try:
                        wrapper = msgpack.unpackb(body, raw=False)
                        tracking_list = wrapper.get('objects', [])
                        with app_state.tracking_data_lock:
                            app_state.tracking_data = tracking_list
                        last_processed_seq = seq
                    except Exception as e:
                        print(f"Failed to decode telemetry: {e}", end='\r')
                    del reassembly[seq]

                # ── Drop timed-out incomplete messages ────────────────────────
                # Keeps last-known-good boxes on screen; never renders partials.
                now = time.time()
                stale_seqs = [
                    s for s, e in reassembly.items()
                    if now - e['first_seen'] > REASSEMBLY_TIMEOUT
                ]
                for stale_seq in stale_seqs:
                    del reassembly[stale_seq]

            except socket.timeout:
                continue   # normal timeout, loop again
            except Exception as e:
                if not app_state.data_thread_stop_event.is_set():
                    print(f"Data receiver error: {e}")
                    time.sleep(1)

# -----------------------------------------------------------------------------------------------
# Target Selection (Command Sender)
# -----------------------------------------------------------------------------------------------
def select_target_by_click(click_pos, tracking_data, app_state):
    """Finds a target at the click position and sends command to Air Pi"""
   
    found_target = False
    new_target_id = None
   
    for data in tracking_data:
        (start_x, start_y, end_x, end_y) = data['bbox']
        if start_x < click_pos[0] < end_x and start_y < click_pos[1] < end_y:
            new_target_id = data['id']
            found_target = True
            break
           
    with app_state.target_lock:
        app_state.current_target_id = new_target_id
       
    if found_target:
        print(f"Clicked new target: ID {new_target_id}")
    else:
        print("Clicked empty space, clearing target.")
       
    # Send command to Air Pi
    send_command_to_air_pi({'target_id': new_target_id})


def send_command_to_air_pi(command_dict):
    """Sends a JSON command to the Air Pi over TCP"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2.0) # 2-second timeout
            s.connect((AIR_PI_IP, COMMAND_PORT))
            command_json = json.dumps(command_dict)
            s.sendall(command_json.encode('utf-8'))
            print(f"Sent command to Air Pi: {command_json}")
    except socket.timeout:
        print(f"Error: Connection to Air Pi command server timed out.")
    except ConnectionRefusedError:
        print(f"Error: Connection to Air Pi command server refused.")
    except Exception as e:
        print(f"Error sending command: {e}")

# -----------------------------------------------------------------------------------------------
# Pygame UI Functions
# -----------------------------------------------------------------------------------------------
def handle_input(events, ui_state, app_state, tracking_data):
    """Handle pygame events"""
    for event in events:
        if event.type == pygame.QUIT:
            ui_state['running'] = False
           
        elif event.type == pygame.MOUSEBUTTONDOWN:
            click_pos = pygame.mouse.get_pos()
            # Handle click in main thread to avoid race conditions
            select_target_by_click(click_pos, tracking_data, app_state)
               
        elif event.type == pygame.KEYDOWN:
            # Clear target
            if event.key == pygame.K_c:
                print("Target cleared by 'C' key")
                with app_state.target_lock:
                    app_state.current_target_id = None
                send_command_to_air_pi({'target_id': None})
                   
            # Quit
            elif event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                ui_state['running'] = False


def render_graphics(screen, frame, tracking_data, current_target_id, ui_state, fonts):
    """Render frame with tracking overlays"""
   
    if frame is None:
        # Display "Waiting for video" message
        screen.fill((30, 30, 30)) # Dark grey background
        text_surface = fonts['main'].render("Waiting for video stream...", True, (255, 255, 255))
        text_rect = text_surface.get_rect(center=screen.get_rect().center)
        screen.blit(text_surface, text_rect)
        pygame.display.flip()
        return

    # Convert numpy array to pygame surface
    # The frame is already RGB, but GStreamer gives it to us rotated.
    # We must rotate it and flip it to be upright.
    frame_surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
   
    screen.blit(frame_surface, (0, 0))
   
    frame_width, frame_height = screen.get_size()
   
    # Draw tracked objects
    for data in tracking_data:
        (start_x, start_y, end_x, end_y) = data['bbox']
       
        is_target = (data['id'] == current_target_id)
       
        # Red for target, green for others
        color = (255, 0, 0) if is_target else (0, 255, 0)
        thickness = 3 if is_target else 2
       
        # Draw bounding box
        pygame.draw.rect(
            screen, color,
            (start_x, start_y, end_x - start_x, end_y - start_y),
            thickness
        )
       
        # Draw label
        label_text = f"ID:{data['id']} {data['label']}"
        if is_target:
            label_text = f"★ {label_text} ★"
       
        text_surface = fonts['small'].render(label_text, True, color, (0,0,0))
        screen.blit(text_surface, (start_x, start_y - 25))
   
    # Draw instructions
    instructions = [
        "Click object to track | C: Clear target | Q/ESC: Quit"
    ]
    y_offset = frame_height - 30
    for instruction in instructions:
        instr_surface = fonts['small'].render(instruction, True, (200, 200, 200), (0, 0, 0))
        screen.blit(instr_surface, (10, y_offset))
        y_offset += 25
   
    pygame.display.flip()

# -----------------------------------------------------------------------------------------------
# Main Function
# -----------------------------------------------------------------------------------------------
def main():
    """Main entry point"""
   
    # Init GStreamer and GLib main loop
    Gst.init(None)
    main_loop = GLib.MainLoop()
   
    print("Initializing ground station...")
   
    app_state = AppState()
   
    # Start GStreamer in separate thread
    gst_thread = threading.Thread(target=run_gstreamer_receiver, args=(app_state, main_loop), daemon=True)
    gst_thread.start()
   
    # Start Data receiver in separate thread
    data_thread = threading.Thread(target=run_data_receiver, args=(app_state,), daemon=True)
    data_thread.start()
   
    # Initialize pygame
    pygame.init()
    # Default size, will be resized on first frame
    screen = pygame.display.set_mode((1280, 720), pygame.RESIZABLE)
    pygame.display.set_caption("Ground Station")
    clock = pygame.time.Clock()
    fonts = {
        'main': pygame.font.Font(None, 32),
        'small': pygame.font.Font(None, 24)
    }
   
    ui_state = { 'running': True }
   
    print("Ground station ready. Waiting for video feed...")
   
    # Main loop (Pygame UI)
    try:
        while ui_state['running']:
            # Check for GStreamer errors
            if app_state.gst_error_event.is_set():
                print("GStreamer error detected, exiting...")
                break
           
            # Get local copies of shared data (thread-safe)
            with app_state.frame_lock:
                local_frame = app_state.pygame_frame.copy() if app_state.pygame_frame is not None else None
           
            with app_state.tracking_data_lock:
                local_tracking_data = app_state.tracking_data.copy()
           
            with app_state.target_lock:
                local_target_id = app_state.current_target_id
           
            # Handle input
            handle_input(pygame.event.get(), ui_state, app_state, local_tracking_data)
           
            # Resize screen if needed
            if local_frame is not None and screen.get_size() != (local_frame.shape[1], local_frame.shape[0]):
                frame_width, frame_height = local_frame.shape[1], local_frame.shape[0]
                screen = pygame.display.set_mode((frame_width, frame_height), pygame.RESIZABLE)
                print(f"Video display resized to: {frame_width}x{frame_height}")
           
            # Render UI
            render_graphics(screen, local_frame, local_tracking_data, local_target_id, ui_state, fonts)
           
            clock.tick(60) # Limit UI framerate
           
    except KeyboardInterrupt:
        print("\nShutdown requested by user (Ctrl+C)...")
       
    finally:
        print("Shutting down...")
       
        # Stop threads
        app_state.gst_error_event.set()
        app_state.data_thread_stop_event.set()
       
        if main_loop.is_running():
            print("Quitting GLib main loop...")
            main_loop.quit()
       
        print("Waiting for threads to join...")
        gst_thread.join(timeout=3)
        data_thread.join(timeout=2)
       
        pygame.quit()
        print("Ground station stopped")
        sys.exit(0)

if __name__ == "__main__":
    main()


