import socket
import struct
import time
import json
import os
import msgpack

# Load configuration from `config.toml` (repo root) if available.
def load_config(path=None):
    path = path or os.environ.get("DRONE_CONFIG", "config.toml")
    if not os.path.exists(path):
        return {}
    try:
        import tomllib as _toml  # Python 3.11+
        with open(path, "rb") as f:
            return _toml.load(f)
    except Exception:
        try:
            import tomllib as _toml  # toml parser
            with open(path, "rb") as f:
                return _toml.load(f)
        except Exception:
            return {}


# Read config and apply defaults
_CFG = load_config()
_NET = _CFG.get("network", {})
RADXA_GADGET_IP = _NET.get("ground_gadget_ip", "10.55.0.1")
VIDEO_STREAM_PORT = int(_NET.get("video_port", 5602)) # Port to listen on for H.264 video (forwarded from drone)
DATA_PORT = int(_NET.get("data_port", 5601)) # Port to listen on for tracking data (forwarded from drone)
COMMAND_PORT = int(_NET.get("command_port", 5603)) # Port to send commands to (Radxa forwards to drone)

# Must match the constants in air_pi.py exactly.
PACKET_HEADER_FMT = "!I H H"   # seq (uint32), frag_id (uint16), frag_count (uint16)
PACKET_HEADER_LEN = struct.calcsize(PACKET_HEADER_FMT)   # 8 byte header 

# Drop any reassembly buffer that has not completed within this time (seconds).
# Default tuned for low-latency telemetry; can be overridden in config.toml
_TEL = _CFG.get("telemetry", {})
REASSEMBLY_TIMEOUT = float(_TEL.get("reassembly_timeout", 0.3))


def run_data_receiver(app_state):
    """
    Listens for fragmented msgpack telemetry packets from the Air Pi.

    Packet header (8 bytes, big-endian):
        seq        (uint32) message sequence number, increments every frame
        frag_id   (uint16) 0-based index of this fragment within the message
        frag_count (uint16) total fragments that make up this message

    Packet-loss behaviour:
        - If some fragments for a message never arrive, the message is silently
          dropped after REASSEMBLY_TIMEOUT seconds.  The display keeps showing
          the last fully-received set of bounding boxes no partial/corrupt
          frames are ever rendered.
        - Out-of-order or duplicate datagrams for already-processed messages
          are discarded using the seq number.
    """
    # {seq: {'frags': {frag_id: bytes}, 'total': int, 'first_seen': float}}
    reassembly = {}
    last_processed_seq = -1

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('', DATA_PORT))
        s.settimeout(1.0)   # allows the stop-event check on every timeout
        print(f"Data receiver listening on port {DATA_PORT}")

        while not app_state.data_thread_stop_event.is_set():
            try:
                data, addr = s.recvfrom(65536)

                # ── Parse header ──────────────────────────────────────────────
                if len(data) < PACKET_HEADER_LEN:
                    continue   # runt packet, ignore

                seq, frag_id, frag_count = struct.unpack(
                    PACKET_HEADER_FMT, data[:PACKET_HEADER_LEN]
                )
                payload = data[PACKET_HEADER_LEN:]

                # ── Discard stale / duplicate datagrams ───────────────────────
                if seq <= last_processed_seq:
                    continue

                # ── Buffer this fragment ──────────────────────────────────────
                if seq not in reassembly:
                    reassembly[seq] = {
                        'frags':      {},
                        'total':      frag_count,
                        'first_seen': time.time(),
                    }
                reassembly[seq]['frags'][frag_id] = payload

                # ── Try to assemble once all fragments have arrived ───────────
                entry = reassembly[seq]
                if len(entry['frags']) == entry['total']:
                    body = b''.join(
                        entry['frags'][i] for i in range(entry['total'])
                    )
                    try:
                        wrapper = msgpack.unpackb(body, raw=False)
                        tracking_list = wrapper.get('objects', [])
                        with app_state.tracking_data_lock:
                            app_state.tracking_data = tracking_list
                        last_processed_seq = seq
                    except Exception as e:
                        print(f"Failed to decode telemetry: {e}", end='\r')
                    del reassembly[seq]

                # ── Drop timed-out incomplete messages ────────────────────────
                # Keeps last-known-good boxes on screen; never renders partials.
                now = time.time()
                stale_seqs = [
                    s for s, e in reassembly.items()
                    if now - e['first_seen'] > REASSEMBLY_TIMEOUT
                ]
                for stale_seq in stale_seqs:
                    del reassembly[stale_seq]

            except socket.timeout:
                continue   # normal timeout, loop again
            except Exception as e:
                if not app_state.data_thread_stop_event.is_set():
                    print(f"Data receiver error: {e}")
                    time.sleep(1)

# -----------------------------------------------------------------------------------------------
# Target Selection (Command Sender)
# -----------------------------------------------------------------------------------------------



def send_command_to_air_pi(command_dict):
    """Sends a JSON command to the Air Pi over TCP"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2.0) # 2-second timeout
            s.connect((RADXA_GADGET_IP, COMMAND_PORT))
            command_json = json.dumps(command_dict)
            s.sendall(command_json.encode('utf-8'))
            print(f"Sent command to Air Pi: {command_json}")
    except socket.timeout:
        print(f"Error: Connection to Air Pi command server timed out.")
    except ConnectionRefusedError:
        print(f"Error: Connection to Air Pi command server refused.")
    except Exception as e:
        print(f"Error sending command: {e}")

