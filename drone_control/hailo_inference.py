import sys
from pathlib import Path
import time
import socket
import msgpack
import hailo
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

from vid_cmd_data import send_telemetry_udp, GROUND_STATION_IP, DATA_PORT

MIN_CONFIDENCE = 0.35
MIN_BBOX_AREA = 1000

script_dir = Path(__file__).parent

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

def on_new_hailo_sample(appsink, app_state): 
    app_state.frame_count = getattr(app_state, 'frame_count', 0) + 1
    if app_state.frame_count % 120 == 0:
        print(f"Processing frame {app_state.frame_count}...", flush=True)   
    
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
    
    net_w, net_h = 1280, 720
    
    with app_state.frame_size_lock:
        if app_state.frame_width != net_w or app_state.frame_height != net_h:
            app_state.frame_width = net_w
            app_state.frame_height = net_h
            print(f"Detected frame size: {net_w}x{net_h}")


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
            
            EDGE_EXCLUSION_MARGIN = 60  # pixels, in 1280x720 space
            
            if xmin < EDGE_EXCLUSION_MARGIN or xmax > (net_w - EDGE_EXCLUSION_MARGIN):
                continue
            if ymin < EDGE_EXCLUSION_MARGIN or ymax > (net_h - EDGE_EXCLUSION_MARGIN):
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
        tracked_objects = app_state.tracker.update(current_detections_info, net_w, net_h)
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
