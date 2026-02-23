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
# GStreamer Pipeline
# -----------------------------------------------------------------------------------------------
def run_gstreamer(main_loop):
    pipeline_str = (
        "libcamerasrc ! "
        "video/x-raw,width=1280,height=720,framerate=30/1 ! "
        "videoconvert ! "
        "video/x-raw,format=I420 ! "
        "x264enc tune=zerolatency bitrate=1500 speed-preset=superfast key-int-max=15 ! "
        "h264parse ! "
        "rtph264pay config-interval=10 pt=96 ! "
        f"udpsink host={GROUND_STATION_IP} port={VIDEO_STREAM_PORT} sync=false async=false"
    )

    print(f"Pipeline:\n{pipeline_str}\n")

    pipeline = None
    try:
        pipeline = Gst.parse_launch(pipeline_str)

        bus = pipeline.get_bus()
        bus.add_signal_watch()

        def on_message(bus, message):
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

        bus.connect("message", on_message)

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
    Gst.init(None)
    main_loop = GLib.MainLoop()

    print("=== Air Pi — Video Smoke Test ===")
    print(f"Streaming to: {GROUND_STATION_IP}:{VIDEO_STREAM_PORT}")
    print("Press Ctrl+C to stop.\n")

    gst_thread = threading.Thread(target=run_gstreamer, args=(main_loop,), daemon=True)
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
        if main_loop.is_running():
            main_loop.quit()
        gst_thread.join(timeout=3)
        print("Air Pi stopped.")
        sys.exit(0)

if __name__ == "__main__":
    main()
