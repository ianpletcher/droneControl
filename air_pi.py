import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import sys
import numpy as np
import hailo # Hailo SDK for AI inference and ROI processing
# Collections and async
from collections import OrderedDict
import asyncio
import threading
from scipy.spatial.distance import cdist
import socket
import json
import msgpack
import struct
import math
import time
import copy 
import tomllib #or tomli?
from mavsdk import System
from pathlib import Path
from pymavlink import mavutil
import logging

# -----------------------------------------------------------------------------------------------
# Detection Filtering Constants
# -----------------------------------------------------------------------------------------------
MIN_CONFIDENCE = 0.3
MIN_BBOX_AREA = 1000

# -----------------------------------------------------------------------------------------------
# State Machine Constants
# -----------------------------------------------------------------------------------------------
TARGET_LOST_HOVER_TIMEOUT = 5.0
HOLD_BEFORE_MANUAL_DURATION = 2.0
STATE_CYCLE_INTERVAL = 0.1


# -----------------------------------------------------------------------------------------------
# Network Configuration
# -----------------------------------------------------------------------------------------------

# TODO Creating Logging for crucial methods 

# FIXME
# Original hard-coded network constants (kept commented so they can be
# restored easily if needed):

# GROUND_STATION_IP = "10.5.0.1"
# VIDEO_STREAM_PORT = 5602
# DATA_PORT = 5601
# COMMAND_PORT = 5603

# Load configuration from `config.toml` if available. Uses stdlib tomllib
# on Python 3.11+ or the `tomli` package on older Pythons.
def load_config(path=None):
    path = path or os.environ.get("DRONE_CONFIG", "config.toml")
    if not os.path.exists(path):
        return {}
    try:
        import tomllib as _toml
        with open(path, "rb") as f:
            return _toml.load(f)
    except Exception:
        try:
            import tomllib as _toml
            with open(path, "rb") as f:
                return _toml.load(f)
        except Exception:
            return {}
        
        
# Read config and apply defaults
_CFG = load_config()
_NET = _CFG.get("network", {})
GROUND_STATION_IP = _NET.get("ground_ip", "10.5.0.1")
VIDEO_STREAM_PORT = int(_NET.get("video_port", 5602))
DATA_PORT = int(_NET.get("data_port", 5601))
COMMAND_PORT = int(_NET.get("command_port", 5603))

# -----------------------------------------------------------------------------------------------
# Telemetry Sender
# -----------------------------------------------------------------------------------------------



PACKET_MAX_BYTES  = 1400 
PACKET_HEADER_FMT = "!IHH"   #seq (I = uint32), frag_id (H = uint16), frag_count (H = uint16)
PACKET_HEADER_LEN = struct.calcsize(PACKET_HEADER_FMT)   # 8 bytes
PACKET_MAX_PAYLOAD = PACKET_MAX_BYTES - PACKET_HEADER_LEN

def send_telemetry_udp(sock, addr, seq, data_bytes):

    # 8 byte header + 1392 byte payload = 1400 byte, under 1446
    # TODO Need to add fragment cap and catch excess frags. Maybe 4 bytes of fragments?

    def _pack_fragments(seq, payload_bytes): 
        if not isinstance(payload_bytes, (bytes, bytearray)): #checks payload object is bytes/bytearray
            raise TypeError("data_bytes must be bytes or bytearray")
        
        total = len(payload_bytes)
        if total == 0:
            frag_count = 1 # empty fragment = datagram (packet)
        else:
            frag_count = int(math.ceil(total / PACKET_MAX_PAYLOAD)) # from bytes divided by payload max

        for frag_id in range(frag_count):
            start = frag_id * PACKET_MAX_PAYLOAD # Index of payload start in array
            payload = payload_bytes[start:start + PACKET_MAX_PAYLOAD] # Slice from array = payload content
            header = struct.pack(PACKET_HEADER_FMT, seq & 0xFFFFFFFF, frag_id, frag_count) 
            yield header + payload

    # Send each fragment and handle errors per-fragment. Return True only if all fragments were sent.
    
    try:
        for frag_id, packet in enumerate(_pack_fragments(seq, data_bytes)):
            try:
                sock.sendto(packet, addr)
            except socket.error as e:
                print(f"send_telemetry_udp: sendto failed for frag {frag_id}: {e}", end='\r')
                return False
    except Exception as e:
        print(f"send_telemetry_udp: error preparing fragments: {e}")
        return False

    return True

# -----------------------------------------------------------------------------------------------
# Drone Command Controller
# -----------------------------------------------------------------------------------------------

class DroneCommandController: # FIXME: rename to something like TargetTracker or DroneController since it also handles target tracking logic, not just command generation
    """Generates velocity commands based on target position in frame"""

    # Control gains and parameters (tuned for stable tracking behavior)
    def __init__(self): 
        # Command gains in m/s, per call
        self.YAW_GAIN = -0.001
        self.UP_DOWN_GAIN = -0.001
        self.FORWARD_GAIN = 0.00001
        # Bounding box target ratio, use this to follow target 
        self.TARGET_BBOX_AREA_RATIO = 0.05
        self.VERTICAL_SETPOINT_RATIO = 0.8
        # Maximum command limits
        self.MAX_YAW_RATE = 10.0
        self.MAX_VERTICAL_VEL = 1.0
        self.MAX_FORWARD_VEL = 2.0

    # Given a bounding box and frame dimensions, compute velocity commands
    def compute_command(self, bbox, frame_width, frame_height):
        if bbox is None or frame_width == 0 or frame_height == 0:
            return 0.0, 0.0, 0.0, "COMMAND: HOVER (No Target Detected)"

        frame_center_x = frame_width / 2
        # For road visibility
        frame_setpoint_y = frame_height * self.VERTICAL_SETPOINT_RATIO

        #Bounding box center and area
        (start_x, start_y, end_x, end_y) = bbox
        dx = end_x - start_x
        dy = end_y - start_y
        bbox_area = dx * dy
        bbox_center_x = start_x + (dx * 0.5)
        bbox_center_y = start_y + (dy * 0.5)

        target_area = (frame_width * frame_height) * self.TARGET_BBOX_AREA_RATIO

        # errors to center the camera on the target and maintain distance based on bbox area
        error_x = bbox_center_x - frame_center_x
        error_y = bbox_center_y - frame_setpoint_y
        error_area = target_area - bbox_area
        # Proportional control for yaw, vertical, and forward velocities
        # np.clip limits (K * error, min, max)

        # FIXME np.clip has more overhead than manual clipping, optimize later if needed. 
        # deadzone would be a good idea to reduce jitter when target is near center. For example, if abs(error_x) < 20 pixels, set yaw_velocity to 0. Similar for vertical and forward with appropriate thresholds.
        yaw_velocity = np.clip(self.YAW_GAIN * error_x, -self.MAX_YAW_RATE, self.MAX_YAW_RATE)
        up_velocity = np.clip(self.UP_DOWN_GAIN * error_y, -self.MAX_VERTICAL_VEL, self.MAX_VERTICAL_VEL)
        forward_velocity = np.clip(self.FORWARD_GAIN * error_area, -self.MAX_FORWARD_VEL, self.MAX_FORWARD_VEL)
        # should change names of up and forward to vertical and horizontal for clarity
 
        # TODO Change to log & send data back to ground station
        command_str = ( #FIXME MASSIVE OVERHEAD WHEN DRONE IS TRACKING
            f"TRACK: FWD={forward_velocity:.2f}m/s | "
            f"UP={up_velocity:.2f}m/s | YAW={yaw_velocity:.2f}°/s"
        ) 

        return forward_velocity, up_velocity, yaw_velocity, command_str

# -----------------------------------------------------------------------------------------------
# Tricking Smoother to Reduce Jitter (not implemented in main)z
# -----------------------------------------------------------------------------------------------
# Implements scaling and deadzone to reduze the movement when a target is relatively centered.
# Helps with flight stability, also reduces the num of commands required, saving a lot of overhead.
class JitterReducer:
    def __init__(self, dead_zone_radius=10, smoothing_factor=0.8):
        # Initialize the jitter reducer.
        self.dead_zone_radius = dead_zone_radius
        self.smoothing_factor = smoothing_factor
        self.previous_position = None

    def is_in_dead_zone(self, x, y, center_x, center_y):
        # Check target is within dead zone.
        distance = math.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
        return distance < self.dead_zone_radius

    def smooth_position(self, current_position):
        # Smooth the position using an exponential moving average.

        if self.previous_position is None:
            self.previous_position = current_position
            return current_position

        smoothed_x = (
            self.smoothing_factor * current_position[0]
            + (1 - self.smoothing_factor) * self.previous_position[0]
        )
        smoothed_y = (
            self.smoothing_factor * current_position[1]
            + (1 - self.smoothing_factor) * self.previous_position[1]
        )

        self.previous_position = (smoothed_x, smoothed_y)
        return smoothed_x, smoothed_y

    def reduce_jitter(self, x, y, center_x, center_y):
        # Reduce jitter for the given position.

        if self.is_in_dead_zone(x, y, center_x, center_y):
            # If in the dead zone, return the center position
            return center_x, center_y
        else:
            # Smooth the position
            return self.smooth_position((x, y))

# -----------------------------------------------------------------------------------------------
# Centroid Tracker
# -----------------------------------------------------------------------------------------------
class CentroidTracker:   #FIXME There are flaws creating new IDs when objects disappear and reappear. Make sure logic is better or same as previous implement.
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

        self.tracker = CentroidTracker(max_disappeared=10) #what is a centroid? 
        self.target_id = None
        self.gst_error_event = threading.Event()

        #Stop even threads
        self.command_thread_stop_event = threading.Event()
        self.control_loop_stop_event = threading.Event()
        self.data_sender_stop_event = threading.Event()
        self.mavsdk_stop_event = threading.Event()
        self.drone_state = "MANUAL" # Mavsdk state
        
        # Initial frame dimensions to be updated by Gstreamer thread
        self.frame_width = 0
        self.frame_height = 0

        # Thread locks for sync access to shared states
        self.target_lock = threading.Lock()
        self.tracker_lock = threading.Lock()
        self.frame_size_lock = threading.Lock()
        self.velocity_lock = threading.Lock()
        self.drone_state_lock = threading.Lock()

        # TODO: New test thread locks for debuffing
        self.drone_state_lock = threading.Lock()
        
        # UDP socket & sequence initialization
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # no SO_SNDBUF increase needed due to fragmentation at send_telemetry_udp level
        self.seq = 0

        # velocity shared state & threads
        self.forward_velocity = 0.0
        self.up_velocity = 0.0
        self.yaw_velocity = 0.0



# -----------------------------------------------------------------------------------------------
# GStreamer Thread
# -----------------------------------------------------------------------------------------------

def run_gstreamer(app_state, main_loop, pipeline_str):
    pipeline = None #default pipeline state to allow for graceful error handling
    try:
        pipeline = Gst.parse_launch(pipeline_str)

        appsink = pipeline.get_by_name("appsink") #parse appsink from pipeline
        if not appsink:
            print("ERROR: Could not find 'appsink' element in pipeline.")
            raise RuntimeError("Missing appsink element")

        appsink.set_property('emit-signals', True) #Enable signal emmisions for new samples
        appsink.set_property('max-buffers', 2)     #Small frame buffer to prevent latency build-up (set to 1 on ground)
        appsink.set_property('drop', True)         #Drop old frames to smoooth feed
        appsink.set_property('async', False)       #Make pull-sample blocking to sync with processing
        appsink.connect('new-sample', on_new_hailo_sample, app_state) # run new hailo sample

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

    buffer = sample.get_buffer() #this is a frame
    if not buffer:
        return Gst.FlowReturn.OK
    
    #FIXME 

    #extract data from frame
    caps = sample.get_caps()
    structure = caps.get_structure(0)
    
    width, height = 640, 640
    with app_state.frame_size_lock:
        if app_state.frame_width != width or app_state.frame_height != height:
            app_state.frame_width = width
            app_state.frame_height = height
            print(f"Detected frame size: {width}x{height}")


    current_detections_info = [] 
    try: #Get interest region and parse the region with the AI inference
        roi = hailo.get_roi_from_buffer(buffer) 
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)


        object_labels = ["car", "truck", "bus", "motorbike", "person"]
        
        # keeps detection if label and confidence meets criteria
        filtered_detections = [det for det in detections if det.get_label() in object_labels and det.get_confidence() >= MIN_CONFIDENCE]

        #FIXME move to config file so we can tune settings easier
        # also incorporate confidence score 
        
        ai_w, ai_h = 640, 640 # AI inference resolution
        
        net_w, net_h = 1280, 720 # Network video resolution
        scale_x = net_w / ai_w
        scale_y = net_h / ai_h


        for det in filtered_detections:
            bbox_raw = det.get_bbox()

            # Convert normalized coordinates to AI frame coordinates
            xmin_ai = int(bbox_raw.xmin() * ai_w)
            ymin_ai = int(bbox_raw.ymin() * ai_h)
            xmax_ai = int(bbox_raw.xmax() * ai_w)
            ymax_ai = int(bbox_raw.ymax() * ai_h)
            
            # Normalized cords for Network stream
            xmin = int(xmin_ai * scale_x)
            ymin = int(ymin_ai * scale_y)
            xmax = int(xmax_ai * scale_x)
            ymax = int(ymax_ai * scale_y)

            if (xmax - xmin) * (ymax - ymin) < MIN_BBOX_AREA:
                continue

            current_detections_info.append({
                'bbox': (xmin, ymin, xmax, ymax),
                'centroid': (int((xmin + xmax) / 2.0), int((ymin + ymax) / 2.0)), # TODO Cast to int later to save processing time, we can afford floats for centroid calculations
                'label': det.get_label(), #comes from object_labels list
                'confidence': det.get_confidence()
            })

    except Exception as e:
        print(f"Error processing Hailo detections: {e}")

    # Gathering data to serialize and send to ground station
    with app_state.tracker_lock:
        tracked_objects = app_state.tracker.update(current_detections_info, width) #not AppState?
        snapshot = tracked_objects.copy()  # cheap shallow snapshot of mapping

    try:
        with app_state.target_lock:
            current_target_id = app_state.target_id

        # Sending to serializer
        # Ensure all values are native Python types (no numpy types) so msgpack
        # serialization is robust on the receiver side. Explicitly coerce fields.
        data_to_send = []
        for oid, data in snapshot.items():
            # Cache locals to reduce repeated lookups
            bbox = data.get('bbox')
            centroid = data.get('centroid')
            label = data.get('label')
            confidence = data.get('confidence')

            item = {
                'id': int(oid),
                'bbox': (int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])) if bbox is not None else None,
                'centroid': (int(centroid[0]), int(centroid[1])) if centroid is not None else None,
                'label': str(label) if label is not None else None,
                'confidence': float(confidence) if confidence is not None else None,
                'is_target': bool(oid == current_target_id),
            }

            # Merge any other small, safe fields that are already serializable
            for k, v in data.items():
                if k in ('bbox', 'centroid', 'label', 'confidence'):
                    continue
                try:
                    if isinstance(v, (int, float, str, bool, list, dict, type(None))):
                        item[k] = v
                except Exception:
                    pass

            data_to_send.append(item)

# -----------------------------------------------------------------------------------------------
# Serialization of Metadata
# -----------------------------------------------------------------------------------------------
# Add more telemetry data as needed, such as frame timestamp, processing latency, etc.

        app_state.seq += 1
        # Build the telemetry envelope
        wrapper = {
            'seq':       app_state.seq,
            'timestamp': time.time(),
            'objects':   data_to_send,
        }

        # Serialise and send (auto-fragmented to stay within MTU)
        msgpack_data = msgpack.packb(wrapper, use_bin_type=True)
        send_telemetry_udp(app_state.data_socket, (GROUND_STATION_IP, DATA_PORT),
                           app_state.seq, msgpack_data)

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
                conn, addr = s.accept() # addr is not used since we only accept from one ground station, but could be used for logging or future features
                with conn:
                    data = conn.recv(1024)
                    if data:
                        command = json.loads(data.decode('utf-8'))
                        if 'target_id' in command:
                            new_target_id = command['target_id']
                            with app_state.drone_state_lock:
                                with app_state.target_lock:
                                    app_state.target_id = new_target_id
                                    if new_target_id is not None:
                                        if app_state.drone_state == "MANUAL":
                                            app_state.drone_state = "TRACKING"
                                            print(f"\n[MANUAL] → TRACKING: Target ID {new_target_id} selected.")
                                        else:
                                            app_state.drone_state = "TRACKING"
                                            print(f"\n[TRACKING] Target switched to ID {new_target_id}.")
                                    else:
                                        app_state.drone_state = "MANUAL"
                                        print("\nTarget cleared by operator. Returning to MANUAL.")


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
    print("Drone control loop running (SIMULATED — no MAVSDK).")

    hover_lost_time = None
    hold_start_time = None
    last_print = "" 

    while not app_state.control_loop_stop_event.is_set():
        try:
            with app_state.drone_state_lock:
                current_state = app_state.drone_state

            with app_state.frame_size_lock:
                frame_w = app_state.frame_width
                frame_h = app_state.frame_height

            with app_state.target_lock:
                target_id = app_state.target_id

            target_bbox = None
            if target_id is not None:
                with app_state.tracker_lock:
                    target_data = app_state.tracker.tracked_objects.get(target_id)
                    if target_data:
                        target_bbox = target_data['bbox']
    # scope issues with variables
    # modify target_bbox to accept None

            if current_state == "MANUAL":
                msg = "STATE: MANUAL | Waiting for target selection..."

            elif current_state == "TRACKING":
                if target_id is None:
                    print("\n[TRACKING] Target cleared. Returning to MANUAL.")
                    with app_state.drone_state_lock:
                        app_state.drone_state = "MANUAL"
                    last_print = ""
                    time.sleep(STATE_CYCLE_INTERVAL)
                    continue

                elif target_bbox is None:
                    print("\n[TRACKING] Target lost. Transitioning to HOVERING.")
                    hover_lost_time = time.time()
                    with app_state.drone_state_lock:
                        app_state.drone_state = "HOVERING"
                    last_print = ""
                    time.sleep(STATE_CYCLE_INTERVAL)
                    continue

                else:
                    forward_vel, up_vel, yaw_rate, command_string = drone_controller.compute_command(
                        target_bbox, frame_w, frame_h
                    )
                    msg = f"[TRACKING] {command_string}"

            elif current_state == "HOVERING":
                elapsed = time.time() - hover_lost_time
                remaining = TARGET_LOST_HOVER_TIMEOUT - elapsed

                if target_bbox is not None:
                    print("\n[HOVERING] Target reacquired. Resuming TRACKING.")
                    hover_lost_time = None
                    with app_state.drone_state_lock:
                        app_state.drone_state = "TRACKING"
                    last_print = ""
                    time.sleep(STATE_CYCLE_INTERVAL)
                    continue

                elif elapsed >= TARGET_LOST_HOVER_TIMEOUT:
                    print("\n[HOVERING] Timeout elapsed. Transitioning to RETURNING.")
                    hold_start_time = time.time()
                    with app_state.drone_state_lock:
                        app_state.drone_state = "RETURNING"
                    last_print = ""
                    time.sleep(STATE_CYCLE_INTERVAL)
                    continue

                else:
                    msg = f"[HOVERING] Searching for target... ({remaining:.1f}s)"

            elif current_state == "RETURNING":
                elapsed = time.time() - hold_start_time
                remaining = HOLD_BEFORE_MANUAL_DURATION - elapsed

                if elapsed >= HOLD_BEFORE_MANUAL_DURATION:
                    print("\n[RETURNING] Hold complete. Returning to MANUAL.")
                    with app_state.target_lock:
                        app_state.target_id = None
                    with app_state.drone_state_lock:
                        app_state.drone_state = "MANUAL"
                    hold_start_time = None
                    last_print = ""
                    time.sleep(STATE_CYCLE_INTERVAL)
                    continue

                else:
                    msg = f"[RETURNING] Holding position... ({remaining:.1f}s)"

            if msg != last_print:
                print(f"  {msg}        ", end='\r')
                last_print = msg

            time.sleep(STATE_CYCLE_INTERVAL)

        except Exception as e:
            if not app_state.control_loop_stop_event.is_set():
                print(f"Drone control loop error: {e}")
                time.sleep(1) 



async def run_drone_control_async(app_state, drone_controller, bbox=None):
        
    drone = System()   #used SERIAL_PORT before, now 
    print(f"Connecting to drone on {COMMAND_PORT}...")

    try:
        await drone.connect(system_address=COMMAND_PORT)

        async for state in drone.core.connection_state():
            if state.is_connected:
                print("Drone connected!")
                break

        print("Waiting for drone to be ready...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("Drone is ready.")
                break

        print("Starting in MANUAL mode. Click a target to engage autonomous tracking.")

        #hover_lost_time = None 
        #hold_start_time = None

        last_command_str = None

        while not app_state.mavsdk_stop_event.is_set():
            try:
                with app_state.drone_state_lock: # current state not implemented correctly
                    current_state = app_state.drone_state

                with app_state.target_lock:
                    target_id = app_state.target_id

                with app_state.frame_size_lock:
                    frame_w = app_state.frame_width
                    frame_h = app_state.frame_height

                target_bbox = None
                if target_id is not None:
                    with app_state.tracker_lock:
                        target_data = app_state.tracker.tracked_objects.get(target_id)
                        if target_data:
                            target_bbox = target_data['bbox']
                # logic might not be correct on drone_controller implementation, need to make sure it can handle target_bbox being None and not send erratic commands.
                forward_velocity, up_velocity, yaw_velocity, command_str = drone_controller.compute_command(
                    target_bbox, frame_w, frame_h
                )
                async with asyncio.Lock():
                    app_state.forward_velocity = forward_velocity                        
                    app_state.up_velocity = up_velocity
                    app_state.yaw_velocity = yaw_velocity
                print(f"Async Control Loop: {command_str}")
                await asyncio.sleep(0.1)
                if command_str != last_command_str:
                    print(f"{command_str}", end='\r')
                    last_command_str = command_str

            except Exception as e:
                if not app_state.mavsdk_stop_event.is_set():
                    print(f"Error in control loop: {e}")
                    await asyncio.sleep(0.1)

    except Exception as e:
        print(f"Drone connection/setup error: {e}")

    finally:
        try:
            await drone.disconnect()
        except Exception as disconnect_error:
            print(f"Error during drone disconnect: {disconnect_error}")
            # target_bbox = None            


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
        "appsink name=appsink emit-signals=true sync=false max-buffers=2 drop=true async=false " + 
        
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
    threads = [
        threading.Thread(name="gstreamer",    target=run_gstreamer,      args=(app_state, main_loop, pipeline_str), daemon=True),
        threading.Thread(name="command",      target=run_command_server, args=(app_state,),                         daemon=True),
        threading.Thread(name="drone_control",target=run_drone_control,  args=(app_state, drone_controller),        daemon=True),
    ]
    for t in threads:
        t.start()

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

        # TTL for thread joining to prevent hanging indefinitely
        print("Waiting for threads to join...")
        for t in threads:
            t.join(timeout=3)

        """        
        gst_thread.join(timeout=3)
        command_thread.join(timeout=2)
        control_thread.join(timeout=2)
        """

        # Close the UDP socket
        app_state.data_socket.close()

        # Final cleanup if necessary
        print("Drone tracker stopped")
        sys.exit(0)

if __name__ == "__main__":
    main()
