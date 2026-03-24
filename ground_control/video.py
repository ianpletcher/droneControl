import numpy as np
import threading
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import logging

from data_cmd import VIDEO_STREAM_PORT


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
       
       # Bus message handler for GStreamer errors and warnings
        bus = pipeline.get_bus()
        bus.add_signal_watch()

        def on_message(bus, message):
            t = message.type
            if t == Gst.MessageType.ERROR:
                err, debug = message.parse_error()
                logging.error(f"GStreamer ERROR: {err}, {debug}")
                app_state.gst_error_event.set()
                main_loop.quit()
            elif t == Gst.MessageType.WARNING:
                err, debug = message.parse_warning()
                logging.warning(f"GStreamer WARNING: {err}, {debug}")
        
        bus.connect("message", on_message)

        appsink = pipeline.get_by_name("appsink")
        if not appsink:
            logging.error("ERROR: Could not find 'appsink' element in pipeline.")
            raise RuntimeError("Missing appsink element")
           
        # Connect our callback
        # appsink.set_property('emit-signals', True) this is already set in pipe
        appsink.connect('new-sample', on_new_video_sample, app_state)
       
        # Set pipeline to PLAYING
        pipeline.set_state(Gst.State.PLAYING)
        logging.info("GStreamer receiver pipeline is running...")
       
        # Run the GLib main loop
        main_loop.run()
       
    except Exception as e:
        logging.error(f"GStreamer error: {e}")
    finally:
        logging.info("GStreamer thread stopping...")
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
   
    # Map the buffer to access pixel data and convert to numpy array
    try:
        (result, map_info) = buffer.map(Gst.MapFlags.READ)
        if result:
             # Create a numpy array that shares memory with the GStreamer buffer
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
        logging.error(f"Error processing video frame: {e}")
    finally:
        buffer.unmap(map_info)
       
    return Gst.FlowReturn.OK
