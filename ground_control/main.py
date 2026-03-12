import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib
# do we not need to specify gst_bin as gst?
import os
import sys
import pygame #rendering UI
import threading

from app_state import AppState
from data_cmd import VIDEO_STREAM_PORT, DATA_PORT, COMMAND_PORT, RADXA_GADGET_IP
from data_cmd import run_data_receiver
from video import run_gstreamer_receiver
from ui import handle_input, render_graphics

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


# Radxa Zero 3W USB Ethernet Gadget IP (usb0 interface, static 10.55.0.1)
# The Radxa forwards all three streams transparently between the drone and this laptop.
""" RADXA_GADGET_IP = "10.55.0.1"         # Radxa USB gadget IP (forwards commands to drone)
VIDEO_STREAM_PORT = 5602         # Port to listen on for H.264 video (forwarded from drone)
DATA_PORT = 5601                 # Port to listen on for tracking data (forwarded from drone)
COMMAND_PORT = 5603              # Port to send commands to (Radxa forwards to drone) """

# -----------------------------------------------------------------------------------------------
# Main Function
# -----------------------------------------------------------------------------------------------
def main():
    """Main entry point"""
   
    # Init GStreamer and GLib main loop
    Gst.init(None)
    main_loop = GLib.MainLoop()
   
    print("Initializing ground station...")
    print("Initializing ground station...")
    print(f"Expecting Radxa USB gadget at: {RADXA_GADGET_IP}")
    print(f"Video  : UDP :{VIDEO_STREAM_PORT}")
    print(f"Data   : UDP :{DATA_PORT}")
    print(f"Command: TCP :{COMMAND_PORT}")
   
    app_state = AppState()
   
    # Start GStreamer in separate thread
    gst_thread = threading.Thread(target=run_gstreamer_receiver, args=(app_state, main_loop), daemon=True)
    gst_thread.start()
   
    # Start Data receiver in separate thread
    data_thread = threading.Thread(target=run_data_receiver, args=(app_state,), daemon=True)
    data_thread.start()

    # Set to use Cocoa video driver for macOS, may need adjustment for other platforms
    os.environ.setdefault('SDL_VIDEODRIVER', 'cocoa')
   
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