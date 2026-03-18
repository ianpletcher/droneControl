from centroid_tracker import CentroidTracker
import threading
import socket

class AppState:
    def __init__(self):

        self.tracker = CentroidTracker(
            max_disappeared=30, 
            tight_distance_ratio=0.2,
            max_distance_ratio=0.4, 
            hit_streak_required=5,
            velocity_decay=0.5,
            edge_margin=50,
            next_id_counter=1,
            max_color_distance = 90.0,
            iou_suppresion_thresh=0.3
            )
        
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

