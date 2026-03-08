#Stripped down version of air_pi for video-only testing
#Removes DroneCommandController, Centroid Tracker, AppState, etc.
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import sys
import time
import threading

# -----------------------------------------------------------------------------------------------
# Network Configuration
# -----------------------------------------------------------------------------------------------
GROUND_STATION_IP = "10.5.0.1"
VIDEO_STREAM_PORT = 5602

# -----------------------------------------------------------------------------------------------
# Test Constants
# -----------------------------------------------------------------------------------------------
TARGET_LOST_HOVER_TIMEOUT = 5.0     
HOLD_BEFORE_MANUAL_DURATION = 2.0   
STATE_CYCLE_INTERVAL = 0.1         

# -----------------------------------------------------------------------------------------------
# State Test
# -----------------------------------------------------------------------------------------------
class TestState:                            
    def __init__(self):
        self.drone_state = "MANUAL"         
        self.target_id = None               
        self.state_lock = threading.Lock()  
        self.stop_event = threading.Event() 
        self.textoverlay = None             
        self.overlay_lock = threading.Lock()

    def set_overlay(self, text):            
        with self.overlay_lock:
            if self.textoverlay is not None:
                self.textoverlay.set_property('text', text)


# -----------------------------------------------------------------------------------------------
#  State Machine Thread
# -----------------------------------------------------------------------------------------------
def run_state_machine(test_state):          
    hover_lost_time = None
    hold_start_time = None

    print("State machine running. Simulating state transitions...")
    print("States will cycle automatically to validate logic.\n")


    def simulate_target_selection():
        time.sleep(3)
        with test_state.state_lock:
            if test_state.drone_state == "MANUAL":
                test_state.target_id = 1
                print("\n[SIM] Target ID 1 selected by operator.")


    def simulate_target_lost():
        time.sleep(8)
        with test_state.state_lock:
            if test_state.drone_state == "TRACKING":
                test_state.target_id = None
                print("\n[SIM] Target lost (removed from tracker).")

    threading.Thread(target=simulate_target_selection, daemon=True).start()
    threading.Thread(target=simulate_target_lost, daemon=True).start()

    while not test_state.stop_event.is_set():
        try:
            with test_state.state_lock:
                current_state = test_state.drone_state
                target_id = test_state.target_id

          
            if current_state == "MANUAL":
                overlay_text = "STATE: MANUAL | Waiting for target selection..."  
                test_state.set_overlay(overlay_text)                               

                if target_id is not None:
                    print("\n[MANUAL] Target selected. Engaging offboard mode.")
                    print("[MANUAL] → TRACKING")
                    hover_lost_time = None
                    with test_state.state_lock:
                        test_state.drone_state = "TRACKING"

            elif current_state == "TRACKING":
                target_found = target_id is not None
                overlay_text = f"STATE: TRACKING | Target ID: {target_id}"       
                test_state.set_overlay(overlay_text)                               

                if target_id is None:
                    print("\n[TRACKING] Target cleared by operator. Returning to MANUAL.")
                    with test_state.state_lock:
                        test_state.drone_state = "MANUAL"

                elif not target_found:
                    print("\n[TRACKING] Target lost. Transitioning to HOVERING.")
                    hover_lost_time = time.time()
                    with test_state.state_lock:
                        test_state.drone_state = "HOVERING"

                else:
                    
                    print(f"  [TRACKING] COMMAND: FWD=0.50m/s | UP=0.10m/s | YAW=1.20deg/s        ", end='\r')

            elif current_state == "HOVERING":
                elapsed = time.time() - hover_lost_time
                remaining = TARGET_LOST_HOVER_TIMEOUT - elapsed
                overlay_text = f"STATE: HOVERING | Target lost — searching... ({remaining:.1f}s)"  
                test_state.set_overlay(overlay_text)                                                

                if target_id is not None:
                    print("\n[HOVERING] Target reacquired. Resuming TRACKING.")
                    hover_lost_time = None
                    with test_state.state_lock:
                        test_state.drone_state = "TRACKING"

                elif elapsed >= TARGET_LOST_HOVER_TIMEOUT:
                    print("\n[HOVERING] Timeout elapsed. Transitioning to RETURNING.")
                    hold_start_time = time.time()
                    with test_state.state_lock:
                        test_state.drone_state = "RETURNING"

                else:
                    print(f"  [HOVERING] Holding position... ({remaining:.1f}s)        ", end='\r')

            elif current_state == "RETURNING":
                elapsed = time.time() - hold_start_time
                remaining = HOLD_BEFORE_MANUAL_DURATION - elapsed
                overlay_text = f"STATE: RETURNING | Holding position... ({remaining:.1f}s)" 
                test_state.set_overlay(overlay_text)                                         

                if elapsed >= HOLD_BEFORE_MANUAL_DURATION:
                    print("\n[RETURNING] Hold complete. Stopping offboard. Returning to MANUAL.")
                    with test_state.state_lock:
                        test_state.target_id = None
                        test_state.drone_state = "MANUAL"
                    hold_start_time = None
                else:
                    print(f"  [RETURNING] Holding position... ({remaining:.1f}s)        ", end='\r')

            time.sleep(STATE_CYCLE_INTERVAL)

        except Exception as e:
            if not test_state.stop_event.is_set():
                print(f"State machine error: {e}")
                time.sleep(1)

# -----------------------------------------------------------------------------------------------
# GStreamer Pipeline
# -----------------------------------------------------------------------------------------------
def run_gstreamer(main_loop, test_state):
    pipeline_str = (
        "libcamerasrc ! "
        "video/x-raw,width=1280,height=720,framerate=30/1 ! "
        "videoconvert ! "
        "textoverlay name=overlay "
        "text='STATE: MANUAL' "
        "valignment=top halignment=left "
        "font-desc='Sans Bold 18' ! "
        "video/x-raw,format=I420 ! "
        "x264enc tune=zerolatency bitrate=1500 speed-preset=superfast key-int-max=15 ! "
        "h264parse ! "
        "rtph264pay config-interval=10 pt=96 ! "
        f"udpsink host={GROUND_STATION_IP} port={VIDEO_STREAM_PORT} sync=false async=false"
    ) #linux command that runs gStreamer pipeline on Raspberry Pi 5

    print(f"Pipeline:\n{pipeline_str}\n") #Printing the pipeline command string for confirmation.

    pipeline = None
    try:
        pipeline = Gst.parse_launch(pipeline_str)
        overlay_element = pipeline.get_by_name("overlay")
        if overlay_element:
            test_state.textoverlay = overlay_element
        else:
            print("Overlay will not function.")


        bus = pipeline.get_bus()
        bus.add_signal_watch()

        def on_message(bus, message): #pipeline message
            t = message.type
            if t == Gst.MessageType.ERROR:
                err, debug = message.parse_error()
                print(f"GStreamer ERROR: {err} | Debug: {debug}")
                main_loop.quit()
            elif t == Gst.MessageType.EOS:
                print("GStreamer: End of stream")
                main_loop.quit()
            elif t == Gst.MessageType.STATE_CHANGED:
                if message.src == pipeline:
                    old, new, _ = message.parse_state_changed()
                    print(f"Pipeline state: {old.value_nick} → {new.value_nick}")

        bus.connect("message", on_message) #Connect message signal to on_message callback function

        ret = pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("ERROR: Could not set pipeline to PLAYING state.")
            sys.exit(1)

        print("GStreamer pipeline running — streaming video to ground station...")
        main_loop.run()

    except Exception as e:
        print(f"GStreamer error: {e}")
    finally:
        print("Stopping pipeline...")
        if pipeline:
            pipeline.set_state(Gst.State.NULL)

# -----------------------------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------------------------
def main():
    Gst.init(None) #initialize gStreamer library
    main_loop = GLib.MainLoop() #loop manager

    print("=== Air Pi — Video Smoke Test ===")
    print(f"Streaming to: {GROUND_STATION_IP}:{VIDEO_STREAM_PORT}")
    print("Press Ctrl+C to stop.\n")

    test_state = TestState()  

    
    sm_thread = threading.Thread(target=run_state_machine, args=(test_state,), daemon=True)
    sm_thread.start()

    
    gst_thread = threading.Thread(target=run_gstreamer, args=(main_loop, test_state), daemon=True)
    gst_thread.start()

    try:
        while True:
            if not gst_thread.is_alive():
                print("GStreamer thread exited.")
                break
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nShutdown requested (Ctrl+C)...")

    finally:
        test_state.stop_event.set()
        if main_loop.is_running():
            main_loop.quit()
        sm_thread.join(timeout=3)
        gst_thread.join(timeout=3)
        print("Air Pi stopped.")
        sys.exit(0)

if __name__ == "__main__":
    main()
