import threading

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