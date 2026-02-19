## Current Hardware Implementation
### Drone:
SBC: Raspberry Pi 5, AI hat+ with Hailo8L Accelerator
Operating System: Debian 13 with WFB-NG installed
Wifi Interface: RTL8812eu with patched driver for WFB-NG

### Ground Station:
SBC: Radxa Zero 3w
Operating System: Debian 11 with WFB-NG installed
Wifi Interface: RTL8812au with patched driver for WFB-NG

## Previous Implementation:

### Drone:
SBC: Raspberry Pi 5, AI hat+ with Hailo8L Accelerator
Operating System: Debian 13

### Ground Station:
SBC: Raspberry Pi 5
Operating System: Debian 13

### Previous Implementation Details:

The main solution we developed represents a significant architectural evolution, migrating all processing from a laptop to a dedicated two-part Single Board Computer (SBC) system. The SBCs are currently both Raspberry Pi 5s. This self-contained architecture, represented by our air_pi.py and ground_pi.py scripts, requires no external laptop. The software architecture is split between an Onboard Computer (Air) and a Ground Station (Ground). The Air SBC is augmented with a Hailo AI accelerator, which we use to manage the onboard inference workload. The Ground SBC, intended to eventually be a Radxa Zero 3W, serves as the operator's interface.

We implemented communication between the two SBCs directly over a local network, utilizing three distinct, multi-threaded data streams. First, we configured the Air SBC with a GStreamer pipeline to capture, encode (H.264), and stream the live video feed to the Ground SBC over UDP (Port 5000). Second, our script on the Air SBC concurrently runs the Hailo AI detection and tracking logic, pickling the resulting tracking data (bounding boxes, IDs, and labels) and sending it to the Ground SBC over a separate UDP channel (Port 5001). Finally, our ground station script sends user commands, specifically the selected target_id, back to the Air SBC as a JSON payload over a TCP connection (Port 5002), which ensures reliable command delivery.

The operational workflow for this solution begins with the operator, who uses a monitor, keyboard, and mouse attached to the Ground SBC. The operator first executes our ground_pi.py script, which initiates the GStreamer video receiver and the Pygame display window, placing the ground station in a "listening" state. The operator then establishes an SSH connection to the Air SBC and launches our air_pi.py script by entering the following commands: cd hailo-rpi5-examples, source setup_env.sh, python air_pi.py. This action starts the camera, the Hailo AI inference, and our data/video streaming pipelines. The video feed, augmented with real-time detection overlays, appears on the Ground SBC's monitor. The operator can then select a target by clicking on any detected object, which prompts our Ground SBC script to send the corresponding id back to the Air SBC, engaging our autonomous tracking logic.

At present, the Air SBC's DroneCommandController class processes the selected target's position and calculates the required velocity commands (forward, up/down, and yaw) to follow it. However, this Air SBC is not yet physically integrated with the flight controller. Therefore, our software operates in a simulation mode: instead of sending MAVSDK commands to the drone, it prints the generated command string to its console. This simulated command is also transmitted as an overlay on the video stream, providing essential visual feedback to the operator on the Ground Station display, confirming that our tracking logic is functioning correctly.

### Previous Implementation Scripts:

#### Drone SBC Script (air_pi.py):

```python
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import sys
import numpy as np
import hailo
from collections import OrderedDict
import threading
from scipy.spatial.distance import cdist
import socket
import json
import pickle
import time
import copy
from pathlib import Path


# -----------------------------------------------------------------------------------------------
# Network Configuration
# -----------------------------------------------------------------------------------------------
GROUND_STATION_IP = "192.168.0.169"
VIDEO_STREAM_PORT = 5000
DATA_PORT = 5001
COMMAND_PORT = 5002


# -----------------------------------------------------------------------------------------------
# Drone Command Controller
# -----------------------------------------------------------------------------------------------
class DroneCommandController:
    """Generates velocity commands based on target position in frame"""


    def __init__(self):
        self.YAW_GAIN = -0.001
        self.UP_DOWN_GAIN = -0.001
        self.FORWARD_GAIN = 0.00001


        self.TARGET_BBOX_AREA_RATIO = 0.05
        self.VERTICAL_SETPOINT_RATIO = 0.8


        self.MAX_YAW_RATE = 10.0
        self.MAX_VERTICAL_VEL = 1.0
        self.MAX_FORWARD_VEL = 2.0


    def compute_command(self, bbox, frame_width, frame_height):
        if bbox is None or frame_width == 0 or frame_height == 0:
            return 0.0, 0.0, 0.0, "COMMAND: HOVER (No Target)"


        frame_center_x = frame_width / 2
        frame_setpoint_y = frame_height * self.VERTICAL_SETPOINT_RATIO


        (start_x, start_y, end_x, end_y) = bbox
        bbox_center_x = (start_x + end_x) / 2
        bbox_center_y = (start_y + end_y) / 2
        bbox_area = (end_x - start_x) * (end_y - start_y)


        target_area = (frame_width * frame_height) * self.TARGET_BBOX_AREA_RATIO


        error_x = bbox_center_x - frame_center_x
        error_y = bbox_center_y - frame_setpoint_y
        error_area = target_area - bbox_area


        yaw_velocity = np.clip(self.YAW_GAIN * error_x, -self.MAX_YAW_RATE, self.MAX_YAW_RATE)
        up_velocity = np.clip(self.UP_DOWN_GAIN * error_y, -self.MAX_VERTICAL_VEL, self.MAX_VERTICAL_VEL)
        forward_velocity = np.clip(self.FORWARD_GAIN * error_area, -self.MAX_FORWARD_VEL, self.MAX_FORWARD_VEL)


        command_str = (
            f"TRACK: FWD={forward_velocity:.2f}m/s | "
            f"UP={up_velocity:.2f}m/s | YAW={yaw_velocity:.2f}°/s"
        )


        return forward_velocity, up_velocity, yaw_velocity, command_str




# -----------------------------------------------------------------------------------------------
# Centroid Tracker
# -----------------------------------------------------------------------------------------------
class CentroidTracker:
    def __init__(self, max_disappeared=30):
        self.tracked_objects = OrderedDict()
        self.disappeared_frames = OrderedDict()
        self.max_disappeared_frames = max_disappeared


    def _get_next_id(self):
        if not self.tracked_objects:
            return 1
        used_ids = set(self.tracked_objects.keys())
        new_id = 1
        while new_id in used_ids:
            new_id += 1
        return new_id


    def update(self, current_detections_info, frame_width):
        if not current_detections_info:
            for object_id in list(self.disappeared_frames.keys()):
                self.disappeared_frames[object_id] += 1
                if self.disappeared_frames[object_id] > self.max_disappeared_frames:
                    self._deregister(object_id)
            return self.tracked_objects


        if not self.tracked_objects:
            for info in current_detections_info:
                self._register(info)
            return self.tracked_objects


        object_ids = list(self.tracked_objects.keys())
        prev_centroids = np.array([data['centroid'] for data in self.tracked_objects.values()])
        input_centroids = np.array([d['centroid'] for d in current_detections_info])


        if prev_centroids.size == 0 or input_centroids.size == 0:
            if prev_centroids.size == 0:
                for info in current_detections_info:
                    self._register(info)
            return self.tracked_objects


        D = cdist(prev_centroids, input_centroids)
        rows = D.min(axis=1).argsort()
        cols = D.argmin(axis=1)[rows]


        used_rows, used_cols = set(), set()
        max_distance = frame_width / 5.0


        for row, col in zip(rows, cols):
            if row in used_rows or col in used_cols or D[row, col] > max_distance:
                continue
            object_id = object_ids[row]
            self.tracked_objects[object_id] = current_detections_info[col]
            self.disappeared_frames[object_id] = 0
            used_rows.add(row)
            used_cols.add(col)


        unused_rows = set(range(D.shape[0])).difference(used_rows)
        for row in unused_rows:
            object_id = object_ids[row]
            self.disappeared_frames[object_id] += 1
            if self.disappeared_frames[object_id] > self.max_disappeared_frames:
                self._deregister(object_id)


        unused_cols = set(range(D.shape[1])).difference(used_cols)
        for col in unused_cols:
            self._register(current_detections_info[col])


        return self.tracked_objects


    def _register(self, detection_info):
        new_id = self._get_next_id()
        self.tracked_objects[new_id] = detection_info
        self.disappeared_frames[new_id] = 0


    def _deregister(self, object_id):
        del self.tracked_objects[object_id]
        del self.disappeared_frames[object_id]




# -----------------------------------------------------------------------------------------------
# Application State
# -----------------------------------------------------------------------------------------------
class AppState:
    def __init__(self):
        self.tracker = CentroidTracker()
        self.target_id = None
        self.gst_error_event = threading.Event()
        self.command_thread_stop_event = threading.Event()
        self.control_loop_stop_event = threading.Event()
        self.data_sender_stop_event = threading.Event()


        self.frame_width = 0
        self.frame_height = 0


        self.target_lock = threading.Lock()
        self.tracker_lock = threading.Lock()
        self.frame_size_lock = threading.Lock()


        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)




# -----------------------------------------------------------------------------------------------
# GStreamer Thread
# -----------------------------------------------------------------------------------------------
def run_gstreamer(app_state, main_loop, pipeline_str):
    pipeline = None
    try:
        pipeline = Gst.parse_launch(pipeline_str)


        appsink = pipeline.get_by_name("appsink")
        if not appsink:
            print("ERROR: Could not find 'appsink' element in pipeline.")
            raise RuntimeError("Missing appsink element")


        appsink.set_property('emit-signals', True)
        appsink.connect('new-sample', on_new_hailo_sample, app_state)


        pipeline.set_state(Gst.State.PLAYING)
        print("GStreamer pipeline is running...")


        main_loop.run()


    except GLib.Error as e:
        print(f"GStreamer GLib.Error: {e}")
    except Exception as e:
        print(f"GStreamer error: {e}")
    finally:
        print("GStreamer thread stopping...")
        if pipeline:
            pipeline.set_state(Gst.State.NULL)
        app_state.gst_error_event.set()


def on_new_hailo_sample(appsink, app_state):
    sample = appsink.emit('pull-sample')
    if not sample:
        return Gst.FlowReturn.OK


    buffer = sample.get_buffer()
    if not buffer:
        return Gst.FlowReturn.OK


    caps = sample.get_caps()
    structure = caps.get_structure(0)
    width = structure.get_value('width')
    height = structure.get_value('height')


    with app_state.frame_size_lock:
        if app_state.frame_width != width or app_state.frame_height != height:
            app_state.frame_width = width
            app_state.frame_height = height
            print(f"Detected frame size: {width}x{height}")


    current_detections_info = []
    try:
        roi = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)


        object_labels = ["car", "truck", "bus", "motorbike", "person"]
        filtered_detections = [det for det in detections if det.get_label() in object_labels]


        # for det in filtered_detections:
        #     bbox_raw = det.get_bbox()
        #     xmin = int(bbox_raw.xmin() * width)
        #     ymin = int(bbox_raw.ymin() * height)
        #     xmax = int(bbox_raw.xmax() * width)
        #     ymax = int(bbox_raw.ymax() * height)


        #     current_detections_info.append({
        #         'bbox': (xmin, ymin, xmax, ymax),
        #         'centroid': (int((xmin + xmax) / 2.0), int((ymin + ymax) / 2.0)),
        #         'label': det.get_label()
        #     })


        # AI inference resolution
        ai_w, ai_h = 640, 640
        # Network video resolution
        net_w, net_h = 1280, 720


        for det in filtered_detections:
            bbox_raw = det.get_bbox()


            # Convert normalized coordinates to AI frame coordinates
            xmin = bbox_raw.xmin() * ai_w
            ymin = bbox_raw.ymin() * ai_h
            xmax = bbox_raw.xmax() * ai_w
            ymax = bbox_raw.ymax() * ai_h


            # Scale to match outgoing 1280x720 stream
            scale_x = net_w / ai_w
            scale_y = net_h / ai_h


            xmin = int(xmin * scale_x)
            ymin = int(ymin * scale_y)
            xmax = int(xmax * scale_x)
            ymax = int(ymax * scale_y)


            current_detections_info.append({
                'bbox': (xmin, ymin, xmax, ymax),
                'centroid': (int((xmin + xmax) / 2.0), int((ymin + ymax) / 2.0)),
                'label': det.get_label()
            })


    except Exception as e:
        print(f"Error processing Hailo detections: {e}")


    with app_state.tracker_lock:
        tracked_objects = app_state.tracker.update(current_detections_info, width)
        tracked_objects_copy = copy.deepcopy(tracked_objects)


    try:
        with app_state.target_lock:
            current_target_id = app_state.target_id


        data_to_send = [
            {'id': oid, **data, 'is_target': (oid == current_target_id)}
            for oid, data in tracked_objects_copy.items()
        ]


        pickled_data = pickle.dumps(data_to_send)
        app_state.data_socket.sendto(pickled_data, (GROUND_STATION_IP, DATA_PORT))


    except socket.error as e:
        print(f"Network send error: {e}", end='\r')
    except Exception as e:
        print(f"Error sending tracking data: {e}")


    return Gst.FlowReturn.OK




# -----------------------------------------------------------------------------------------------
# Ground Station Command Receiver Thread
# -----------------------------------------------------------------------------------------------
def run_command_server(app_state):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', COMMAND_PORT))
        s.listen()
        s.settimeout(1.0)
        print(f"Command server listening on port {COMMAND_PORT}")


        while not app_state.command_thread_stop_event.is_set():
            try:
                conn, addr = s.accept()
                with conn:
                    data = conn.recv(1024)
                    if data:
                        command = json.loads(data.decode('utf-8'))
                        if 'target_id' in command:
                            new_target_id = command['target_id']
                            with app_state.target_lock:
                                app_state.target_id = new_target_id
                            print(f"\nTarget set by ground station: ID {new_target_id}")


            except socket.timeout:
                continue
            except json.JSONDecodeError:
                print("Received invalid command (not JSON)")
            except Exception as e:
                if not app_state.command_thread_stop_event.is_set():
                    print(f"Command server error: {e}")


# -----------------------------------------------------------------------------------------------
# Drone Control Loop (MAVLink) Thread
# -----------------------------------------------------------------------------------------------
def run_drone_control(app_state, drone_controller):
    print("Drone control loop running (TEST MODE).")


    while not app_state.control_loop_stop_event.is_set():
        try:
            with app_state.frame_size_lock:
                frame_w = app_state.frame_width
                frame_h = app_state.frame_height


            target_bbox = None
            with app_state.target_lock:
                target_id = app_state.target_id


            if target_id is not None:
                with app_state.tracker_lock:
                    target_data = app_state.tracker.tracked_objects.get(target_id)
                    if target_data:
                        target_bbox = target_data['bbox']


            forward_vel, up_vel, yaw_rate, command_string = drone_controller.compute_command(
                target_bbox, frame_w, frame_h
            )


            print(f"  {command_string}        ", end='\r')


            time.sleep(0.1)


        except Exception as e:
            if not app_state.control_loop_stop_event.is_set():
                print(f"Drone control loop error: {e}")
                time.sleep(1)




# -----------------------------------------------------------------------------------------------
# Main Function
# -----------------------------------------------------------------------------------------------
def main():
    Gst.init(None)
    main_loop = GLib.MainLoop()


    print("Initializing drone object tracker...")


    app_state = AppState()
    drone_controller = DroneCommandController()


    # --- Post-process library location ---
    postprocess_so = "/usr/lib/aarch64-linux-gnu/hailo/tappas/post_processes/libyolo_hailortpp_post.so"


    if not Path(postprocess_so).exists():
        print(f"ERROR: Post-process library not found at {postprocess_so}")
        print("Please check your Hailo TAPPAS installation")
        sys.exit(1)


    print(f"Using post-process library: {postprocess_so}")


    # --- Find model ---
    script_dir = Path(__file__).resolve().parent  # Define script directory


    def find_best_model():
        """Find the best available COCO detection model for this device"""


        # Detect Hailo architecture
        import subprocess
        is_hailo8l = True  # Default assumption
        try:
            result = subprocess.run(['hailortcli', 'fw-control', 'identify'],
                                  capture_output=True, text=True, timeout=5)
            is_hailo8l = 'HAILO8L' in result.stdout
            arch_name = "Hailo-8L (13 TOPS)" if is_hailo8l else "Hailo-8 (26 TOPS)"
            print(f"Detected device: {arch_name}")
        except Exception as e:
            print(f"Could not detect architecture, assuming Hailo-8L: {e}")


        # Choose appropriate models for architecture
        if is_hailo8l:
            model_priority = [
                # Try rpicam-apps models first (these work!)
                "/usr/share/hailo-models/yolov8s_h8l.hef",
                "/usr/share/hailo-models/yolov6n_h8l.hef",
                # Try downloaded models
                str(Path.home() / "hailo_models_8l/yolov8s_h8l.hef"),
                # Try hailo-rpi5-examples (these might be wrong architecture)
                "yolov8s.hef",
                "yolov6n.hef",
                "yolov5s.hef",
            ]
        else:  # Hailo-8
            model_priority = [
                "yolov8m.hef",
                "yolov8s.hef",
            ]


        search_paths = [
            Path.home() / "hailo-rpi5-examples/resources",
            Path("/usr/share/hailo-models"),
            script_dir,
        ]


        for path in search_paths:
            for model_name in model_priority:
                model_path = path / model_name
                if model_path.exists():
                    print(f"Found model: {model_path}")
                    return model_path


        print("ERROR: No compatible model found!")
        print("Run: cd ~/hailo-rpi5-examples && ./download_resources.sh --all")
        sys.exit(1)


    model_path = find_best_model()


    # --- FIXED PIPELINE based on official Hailo examples ---
    # Key changes:
    # 1. Scale to 640x640 BEFORE hailonet
    # 2. Add hailofilter with post-process .so file and thresholds
    # 3. Use proper queue structure with leaky queues
    # 4. RGB format for AI processing
    # 5. Increased UDP buffer and async sending


    def QUEUE(name, max_size_buffers=3, leaky="downstream"):
        """Create a leaky queue to prevent blocking"""
        return f"queue name={name} leaky={leaky} max-size-buffers={max_size_buffers} max-size-bytes=0 max-size-time=0 ! "


    pipeline_str = (
        # Camera source at native resolution
        "libcamerasrc ! "
        "video/x-raw,width=1920,height=1080,framerate=30/1,format=NV12 ! "


        # Split into two paths
        "tee name=t allow-not-linked=true "  # CRITICAL: allow-not-linked prevents blocking


        # --- Path 1: AI Processing ---
        "t. ! " +
        QUEUE("queue_ai_scale", max_size_buffers=2, leaky="downstream") +
        "videoscale ! "
        "video/x-raw,width=640,height=640 ! "  # Scale to model input size
        "videoconvert ! "
        "video/x-raw,format=RGB ! "  # Convert to RGB for AI
        f"hailonet hef-path={model_path} is-active=true ! " +
        QUEUE("queue_hailofilter", max_size_buffers=2, leaky="downstream") +
        # Note: hailofilter thresholds will be applied in Python filtering below
        f"hailofilter so-path={postprocess_so} qos=false ! " +
        QUEUE("queue_appsink", max_size_buffers=2, leaky="downstream") +
        "appsink name=appsink emit-signals=true sync=false max-buffers=2 drop=true async=false "  # async=false prevents blocking


        # --- Path 2: Network Stream ---
        "t. ! " +
        QUEUE("queue_network", max_size_buffers=5, leaky="downstream") +
        "videoscale ! video/x-raw,width=1280,height=720 ! "  # Reduce resolution for network
        "videoconvert ! "
        "video/x-raw,format=I420 ! "
        "x264enc tune=zerolatency bitrate=1500 speed-preset=superfast key-int-max=15 ! "  # Force keyframe every 15 frames (0.5s)
        "h264parse ! "  # CRITICAL: Parse H264 stream before RTP payload
        "rtph264pay config-interval=10 pt=96 ! "  # Send SPS/PPS every 10 packets
        f"udpsink host={GROUND_STATION_IP} port={VIDEO_STREAM_PORT} sync=false async=false"
    )


    print(f"\nUsing pipeline:\n{pipeline_str}\n")


    # --- Start All Threads ---
    gst_thread = threading.Thread(target=run_gstreamer, args=(app_state, main_loop, pipeline_str), daemon=True)
    gst_thread.start()


    command_thread = threading.Thread(target=run_command_server, args=(app_state,), daemon=True)
    command_thread.start()


    control_thread = threading.Thread(target=run_drone_control, args=(app_state, drone_controller), daemon=True)
    control_thread.start()


    try:
        while True:
            if app_state.gst_error_event.is_set():
                print("Main thread: Error detected, exiting...")
                break
            time.sleep(1)


    except KeyboardInterrupt:
        print("\nShutdown requested by user (Ctrl+C)...")


    finally:
        print("Shutting down...")


        app_state.gst_error_event.set()
        app_state.command_thread_stop_event.set()
        app_state.control_loop_stop_event.set()
        app_state.data_sender_stop_event.set()


        if main_loop.is_running():
            print("Quitting GLib main loop...")
            main_loop.quit()


        print("Waiting for threads to join...")
        gst_thread.join(timeout=3)
        command_thread.join(timeout=2)
        control_thread.join(timeout=2)


        app_state.data_socket.close()


        print("Drone tracker stopped")
        sys.exit(0)




if __name__ == "__main__":
    main()
```

#### Ground Station Script (ground_station.py)

```python
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib
import os
import sys
import pygame
import numpy as np  # <-- ADDED THIS MISSING IMPORT
import threading
import socket
import json
import pickle
import time


# -----------------------------------------------------------------------------------------------
# Network Configuration
# -----------------------------------------------------------------------------------------------
# NOTE: This Pi's IP is 192.168.0.169.
# We are listening on all interfaces (0.0.0.0).
AIR_PI_IP = "192.168.0.170"      # IP of the Air Pi (for sending commands to)
VIDEO_STREAM_PORT = 5000         # Port to listen on for H.264 video
DATA_PORT = 5001                 # Port to listen on for tracking data
COMMAND_PORT = 5002              # Port on the Air Pi to send commands to




# -----------------------------------------------------------------------------------------------
# Application State
# -----------------------------------------------------------------------------------------------
class AppState:
    """Manages shared state between GStreamer and UI threads"""
   
    def __init__(self):
        self.pygame_frame = None       # The most recent numpy frame
        self.tracking_data = []      # The most recent tracking data
        self.last_click_pos = None     # (x, y) tuple of the last click
        self.current_target_id = None  # The ID we are currently tracking
        self.gst_error_event = threading.Event() # Set if GStreamer fails
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
   
    pipeline = None
    try:
        pipeline = Gst.parse_launch(pipeline_str)
       
        # Find the appsink
        appsink = pipeline.get_by_name("appsink")
        if not appsink:
            print("ERROR: Could not find 'appsink' element in pipeline.")
            raise RuntimeError("Missing appsink element")
           
        # Connect our callback
        appsink.set_property('emit-signals', True)
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
def run_data_receiver(app_state):
    """Listens for UDP packets containing pickled tracking data"""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', DATA_PORT)) # Listen on all interfaces
        s.settimeout(1.0) # Set timeout so the loop can check for stop event
        print(f"Data receiver listening on port {DATA_PORT}")
       
        while not app_state.data_thread_stop_event.is_set():
            try:
                data, addr = s.recvfrom(65536) # Max UDP packet size
                if data:
                    tracking_list = pickle.loads(data)
                   
                    with app_state.tracking_data_lock:
                        app_state.tracking_data = tracking_list
                       
            except socket.timeout:
                continue # Normal timeout, loop again
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
        wait_text = "Waiting for video stream..."
        text_surface = fonts['main'].render(wait_text, True, (255, 255, 255))
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
```

## Current Goal:

* Determine best path forward for networking/signal transfer
* Modify existing scripts to accommodate new hardware layout
