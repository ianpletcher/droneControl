import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib
import os
import sys
import pygame
import numpy as np
import threading
import socket
import json
import pickle
import time


# -----------------------------------------------------------------------------------------------
# Network Configuration
# -----------------------------------------------------------------------------------------------
# Radxa Zero 3W USB Ethernet Gadget IP (usb0 interface, static 10.55.0.1)
# The Radxa forwards all three streams transparently between the drone and this laptop.
RADXA_GADGET_IP = "10.55.0.1"         # Radxa USB gadget IP (forwards commands to drone)
VIDEO_STREAM_PORT = 5602         # Port to listen on for H.264 video (forwarded from drone)
DATA_PORT = 5601                 # Port to listen on for tracking data (forwarded from drone)
COMMAND_PORT = 5603              # Port to send commands to (Radxa forwards to drone)


# -----------------------------------------------------------------------------------------------
# Application State
# -----------------------------------------------------------------------------------------------
class AppState:
    """Manages shared state between GStreamer and UI threads"""

    def __init__(self):
        self.pygame_frame = None       # The most recent numpy frame
        self.tracking_data = []        # The most recent tracking data
        self.last_click_pos = None     # (x, y) tuple of the last click
        self.current_target_id = None  # The ID we are currently tracking
        self.gst_error_event = threading.Event()    # Set if GStreamer fails
        self.data_thread_stop_event = threading.Event()

        # Thread locks
        self.frame_lock = threading.Lock()
        self.tracking_data_lock = threading.Lock()
        self.click_lock = threading.Lock()
        self.target_lock = threading.Lock()


# -----------------------------------------------------------------------------------------------
# GStreamer Video Receiver Thread
# -----------------------------------------------------------------------------------------------
def run_gstreamer_receiver(app_state, main_loop):
    """Run GStreamer in separate thread to receive and display video"""

    pipeline_str = (
        f"udpsrc port={VIDEO_STREAM_PORT} caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96\" ! "
        "rtpjitterbuffer latency=50 drop-on-latency=true ! "
        "rtph264depay ! "
        "h264parse ! "
        "avdec_h264 ! "
        "videoscale ! video/x-raw,width=1280,height=720 ! "
        "videoconvert ! "
        "video/x-raw,format=RGB ! "
        "appsink name=appsink emit-signals=true sync=false max-buffers=1 drop=true"
    )

    pipeline = None
    try:
        pipeline = Gst.parse_launch(pipeline_str)

        # Bus message handler for GStreamer errors and warnings
        bus = pipeline.get_bus()
        bus.add_signal_watch()

        def on_message(bus, message):
            t = message.type
            if t == Gst.MessageType.ERROR:
                err, debug = message.parse_error()
                print(f"GStreamer ERROR: {err}, {debug}")
                app_state.gst_error_event.set()
                main_loop.quit()
            elif t == Gst.MessageType.WARNING:
                err, debug = message.parse_warning()
                print(f"GStreamer WARNING: {err}, {debug}")
        
        bus.connect("message", on_message)

        # Connect the appsink to fetch video frames
        appsink = pipeline.get_by_name("appsink")
        if not appsink:
            print("ERROR: Could not find 'appsink' element in pipeline.")
            raise RuntimeError("Missing appsink element")
        
        # Configure appsink to emit signals and connect callback for new video samples
        appsink.set_property('emit-signals', True)
        appsink.connect('new-sample', on_new_video_sample, app_state)

        pipeline.set_state(Gst.State.PLAYING)
        print("GStreamer receiver pipeline is running...")

        main_loop.run()

    # Exception handling for GStreamer thread
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

    # Extract buffer
    buffer = sample.get_buffer()
    if not buffer:
        return Gst.FlowReturn.OK

    # Get video frame dimensions from GStreamer caps
    caps = sample.get_caps()
    structure = caps.get_structure(0)
    width = structure.get_value('width')
    height = structure.get_value('height')

    try:
        # Map the buffer to access pixel data and convert to numpy array
        (result, map_info) = buffer.map(Gst.MapFlags.READ)
        if result:
            # Create a numpy array that shares memory with the GStreamer buffer
            numpy_frame = np.ndarray(
                (height, width, 3),
                buffer=map_info.data,
                dtype=np.uint8
            )
            # Update the shared frame in app_state with thread safety
            with app_state.frame_lock:
                app_state.pygame_frame = numpy_frame.copy()

    except Exception as e:
        print(f"Error processing video frame: {e}")
    finally:
        buffer.unmap(map_info)

    return Gst.FlowReturn.OK


# -----------------------------------------------------------------------------------------------
# Tracking Data Receiver Thread
# -----------------------------------------------------------------------------------------------
def run_data_receiver(app_state):
    """Listens for UDP packets containing pickled tracking data"""
    # Drone sends pickled list of dicts for tracking data
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', DATA_PORT))
        s.settimeout(1.0)
        print(f"Data receiver listening on port {DATA_PORT}")

        while not app_state.data_thread_stop_event.is_set():
            try:
                data, addr = s.recvfrom(65536)
                if data:
                    tracking_list = pickle.loads(data)
                    with app_state.tracking_data_lock:
                        app_state.tracking_data = tracking_list

            except socket.timeout:
                continue
            except pickle.UnpicklingError:
                print("Received corrupted tracking data packet", end='\r')
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
        # Setting starting and ending coordinates of bounding box from tracking data
        (start_x, start_y, end_x, end_y) = data['bbox']
        
        # If the click position is within the bounding box, we have found a target
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

    send_command_to_air_pi({'target_id': new_target_id})


def send_command_to_air_pi(command_dict):
    """Sends a JSON command to the Air Pi over TCP via Radxa forwarder"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2.0)
            s.connect((RADXA_GADGET_IP, COMMAND_PORT))
            command_json = json.dumps(command_dict)
            s.sendall(command_json.encode('utf-8'))
            print(f"Sent command: {command_json}")
    except socket.timeout:
        print(f"Error: Command timed out. Is the Radxa USB gadget connected?")
    except ConnectionRefusedError:
        print(f"Error: Command refused. Is the Radxa forwarder running on port {COMMAND_PORT}?")
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
            select_target_by_click(click_pos, tracking_data, app_state)

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_c:
                print("Target cleared by 'C' key")
                with app_state.target_lock:
                    app_state.current_target_id = None
                send_command_to_air_pi({'target_id': None})

            elif event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                ui_state['running'] = False


def render_graphics(screen, frame, tracking_data, current_target_id, ui_state, fonts):
    """Render frame with tracking overlays"""

    if frame is None:
        screen.fill((30, 30, 30))
        wait_text = "Waiting for video stream... (Radxa USB connected?)"
        text_surface = fonts['main'].render(wait_text, True, (255, 255, 255))
        text_rect = text_surface.get_rect(center=screen.get_rect().center)
        screen.blit(text_surface, text_rect)
        pygame.display.flip()
        return

    # swapaxes handles GStreamer's row-major ordering
    frame_surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
    screen.blit(frame_surface, (0, 0))

    frame_width, frame_height = screen.get_size()

    for data in tracking_data:
        (start_x, start_y, end_x, end_y) = data['bbox']
        is_target = (data['id'] == current_target_id)
        color = (255, 0, 0) if is_target else (0, 255, 0)
        thickness = 3 if is_target else 2

        pygame.draw.rect(
            screen, color,
            (start_x, start_y, end_x - start_x, end_y - start_y),
            thickness
        )

        label_text = f"ID:{data['id']} {data['label']}"
        if is_target:
            label_text = f"★ {label_text} ★"

        text_surface = fonts['small'].render(label_text, True, color, (0, 0, 0))
        screen.blit(text_surface, (start_x, start_y - 25))

    instructions = ["Click object to track | C: Clear target | Q/ESC: Quit"]
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

    Gst.init(None)
    main_loop = GLib.MainLoop()

    print("Initializing ground station...")
    print(f"Expecting Radxa USB gadget at: {RADXA_GADGET_IP}")
    print(f"Video  : UDP :{VIDEO_STREAM_PORT}")
    print(f"Data   : UDP :{DATA_PORT}")
    print(f"Command: TCP :{COMMAND_PORT}")

    app_state = AppState()

    gst_thread = threading.Thread(target=run_gstreamer_receiver, args=(app_state, main_loop), daemon=True)
    gst_thread.start()

    data_thread = threading.Thread(target=run_data_receiver, args=(app_state,), daemon=True)
    data_thread.start()

    # Set to use Cocoa video driver for macOS, may need adjustment for other platforms
    os.environ.setdefault('SDL_VIDEODRIVER', 'cocoa')

    pygame.init()
    screen = pygame.display.set_mode((1280, 720), pygame.RESIZABLE)
    pygame.display.set_caption("Drone Ground Station")
    clock = pygame.time.Clock()
    fonts = {
        'main': pygame.font.Font(None, 32),
        'small': pygame.font.Font(None, 24)
    }

    ui_state = {'running': True}

    print("Ground station ready. Waiting for video feed...")

    try:
        while ui_state['running']:
            if app_state.gst_error_event.is_set():
                print("GStreamer error detected, exiting...")
                break

            with app_state.frame_lock:
                local_frame = app_state.pygame_frame.copy() if app_state.pygame_frame is not None else None

            with app_state.tracking_data_lock:
                local_tracking_data = app_state.tracking_data.copy()

            with app_state.target_lock:
                local_target_id = app_state.current_target_id

            handle_input(pygame.event.get(), ui_state, app_state, local_tracking_data)

            if local_frame is not None and screen.get_size() != (local_frame.shape[1], local_frame.shape[0]):
                frame_width, frame_height = local_frame.shape[1], local_frame.shape[0]
                screen = pygame.display.set_mode((frame_width, frame_height), pygame.RESIZABLE)
                print(f"Video display resized to: {frame_width}x{frame_height}")

            render_graphics(screen, local_frame, local_tracking_data, local_target_id, ui_state, fonts)

            clock.tick(60)

    except KeyboardInterrupt:
        print("\nShutdown requested by user (Ctrl+C)...")

    finally:
        print("Shutting down...")
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