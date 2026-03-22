import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
from pathlib import Path
import time
import socket
import struct
import math
import json
import os
import asyncio
import logging
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from pymavlink import mavutil

logging.basicConfig(level=logging.INFO)

def load_config(path=None):
    path = path or os.environ.get("DRONE_CONFIG", "config.toml")
    if not os.path.exists(path):
        return {}
    try:
        import tomllib as _toml
        with open(path, "rb") as f:
            return _toml.load(f)
        except Exception:
            return {}

_CFG = load_config()
_NET = _CFG.get("network", {})
GROUND_STATION_IP = _NET.get("ground_ip", "10.5.0.1")
VIDEO_STREAM_PORT = int(_NET.get("video_port", 5602))
DATA_PORT = int(_NET.get("data_port", 5601))
COMMAND_PORT = int(_NET.get("command_port", 5603))
DRONE_SYSTEM_ADDRESS = _NET.get("drone_system_address", "serial:///dev/ttyAMA0:57600")


# -----------------------------------------------------------------------------------------------
# GStreamer Thread
# -----------------------------------------------------------------------------------------------
def run_gstreamer(app_state, main_loop, pipeline_str):
    from hailo_inference import on_new_hailo_sample
    
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
                        logging.info(f"Received command from {addr}: {data.decode('utf-8')}")
                        command = json.loads(data.decode('utf-8'))
                        if 'target_id' in command:
                            new_target_id = command['target_id']
                            with app_state.target_lock:
                                app_state.target_id = new_target_id
                                with app_state.drone_state_lock:
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
                    else:
                        logging.info("Received empty command.")
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

TARGET_LOST_HOVER_TIMEOUT = 5.0
HOLD_BEFORE_MANUAL_DURATION = 2.0
STATE_CYCLE_INTERVAL = 0.1

def run_drone_control(app_state, drone_controller):
    asyncio.run(run_drone_control_async(app_state, drone_controller))


async def run_drone_control_async(app_state, drone_controller, bbox=None):
        
    drone = System()   #used SERIAL_PORT before, now 
    print(f"Connecting to drone via MAVLink on {DRONE_SYSTEM_ADDRESS}...")


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
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        offboard_active = False


        last_command_str = None

       while not app_state.control_loop_stop_event.is_set():
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

                if current_state == "MANUAL":
                    if offboard_active:
                        try:
                            await drone.offboard.stop()
                            offboard_active = False
                            print("Offboard stopped — MANUAL mode.")
                        except OffboardError as e:
                            print(f"Failed to stop offboard: {e}")
                    await asyncio.sleep(STATE_CYCLE_INTERVAL)
                    continue
                
                forward_velocity, up_velocity, yaw_velocity, command_str = drone_controller.compute_command(
                    target_bbox, frame_w, frame_h )
               
                with app_state.drone_state_lock:
                    app_state.forward_velocity = forward_velocity
                    app_state.up_velocity      = up_velocity
                    app_state.yaw_velocity     = yaw_velocity
 
             
                if not offboard_active:
                    try:
                        await drone.offboard.start()
                        offboard_active = True
                        print("Offboard started — TRACKING mode.")
                    except OffboardError as e:
                        print(f"Failed to start offboard: {e}")
                        await asyncio.sleep(STATE_CYCLE_INTERVAL)
                        continue
 
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(forward_velocity, 0.0, -up_velocity, yaw_velocity)
                )
 
                if command_str != last_command_str:
                    print(f"{command_str}", end='\r')
                    last_command_str = command_str
 
                await asyncio.sleep(STATE_CYCLE_INTERVAL)
 
            except Exception as e:
                if not app_state.control_loop_stop_event.is_set():
                    print(f"Error in control loop: {e}")
                    await asyncio.sleep(STATE_CYCLE_INTERVAL)
 
    except Exception as e:
        print(f"Drone connection/setup error: {e}")
 
    finally:
        print("Drone control loop shutting down...")
        if offboard_active:
            try:
                await drone.offboard.stop()
            except Exception:
                pass
        try:
            await drone.disconnect()
        except Exception as e:
            print(f"Error during drone disconnect: {e}")
