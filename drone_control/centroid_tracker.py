import numpy as np
from scipy.spatial.distance import cdist
from collections import OrderedDict
import logging
import cv2 as cv

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class CentroidTracker:
    def __init__(
        self,
        max_disappeared=30,
        tight_distance_ratio=0.2,
        max_distance_ratio=0.4,
        hit_streak_required=5,
        velocity_decay=0.5,
        edge_margin=50,
        next_id_counter=1,
        max_color_distance=90.0,
        iou_suppresion_thresh=0.3
    ):
        # Confirmed tracks, keyed by object ID with detection info dicts
        self.tracked_objects = OrderedDict() # Mapping object ids to detection info dicts
        self.disappeared_frames = OrderedDict() # Mapping object ids to missed frame counts
        self.velocities = OrderedDict() # Mapping object ids to velocity tuples
        self.colors = OrderedDict() # Mapping confirmed tracks to color data from frame

        # Tentative tracks, keyed by tentative id with detection info dicts and hit counts
        self._tentative_objects = OrderedDict()  # Mapping tent ids to detection info dicts
        self._tentative_hits    = OrderedDict()  # Mapping tent ids to consecutive hit counts
        self._tentative_id_seq  = 0              # Negative ID sequence for tentative tracks to avoid collision with confirmed IDs


        self.max_distance_ratio  = max_distance_ratio # Max distance from last centroid to consider a match, ratio of frame width
        self.tight_distance_ratio = tight_distance_ratio
        self.max_color_distance = max_color_distance
        self.hit_streak_required = hit_streak_required # Consecutive matches required to append tentative match to confirmed tracks
        self.max_disappeared_frames = max_disappeared  # max_disappeared: frames a confirmed track can go unmatched before deregistration
        self.iou_suppresion_thresh = iou_suppresion_thresh
        self.velocity_decay = velocity_decay # Per frame decay factor for velocity to prevent drift during occlusion, 0.5 means velocity halves each unmatched frame
        self.edge_margin = edge_margin # Margin in pixels where velocity influence is reduced to cleanly stop tracking
        self.next_id_counter = next_id_counter # Incrementing id counter to give each object a unique ID

    def update_all_detections(self, current_detections_info, frame_width, frame_height):
        """
        Main update step. Call once per frame with all filtered detections.

        Matching runs in two passes:
          Pass 1 — match detections against confirmed tracks (priority)
          Pass 2 — match remaining detections against tentative tracks
          Remainder — create new tentative entries

        Returns only confirmed tracked_objects (same dict as before)
        
        :param current_detections_info: list of detection info dicts with at least 'centroid' key
        :param frame_width: width of the video frame (for distance normalization)
        :param frame_height: height of the video frame (for distance normalization)
        :return: dict of confirmed tracked objects with their detection info
        """
        max_distance = frame_width * self.max_distance_ratio
        tight_distance = frame_width * self.tight_distance_ratio
        
        if not current_detections_info: # If there are no detections, age all confirmed and tentative tracks and return
            self._age_confirmed(set())
            self._age_tentative(set())
            return self.tracked_objects

        input_centroids = np.array([d['centroid'] for d in current_detections_info]) # Current frame centroid vectors

        # First pass, match against confirmed tracks with velocity-predicted centroids to handle motion
        unmatched_detection_cols = set(range(len(current_detections_info)))

        if self.tracked_objects:
            confirmed_ids  = list(self.tracked_objects.keys()) 
            predicted      = self._build_predicted_centroids(confirmed_ids, frame_width, frame_height)
            prev_centroids = np.array(predicted) # Predicted centroid vectors for confirmed tracks based on velocity

            distance = cdist(prev_centroids, input_centroids) # Distance matrix between predicted centroids and current detections
            rows = distance.min(axis=1).argsort() # Sort confirmed tracks by closets distance
            cols = distance.argmin(axis=1)[rows] # Get index of closest detection for each confirmed track in sorted order

            matched_confirmed_rows = set()
            matched_detection_cols = set()

            for row, col in zip(rows, cols):
                logging.debug(f"{row}, {col}")
                object_id = confirmed_ids[row]
                if row in matched_confirmed_rows or col in matched_detection_cols: # when a match has already been made, skip
                    continue
                
                is_within_tight_distance = distance[row, col] <= tight_distance
                is_within_max_distance = distance[row, col] <= max_distance
                is_similar_color = self._calculate_color_distance(object_id, current_detections_info[col]) <= self.max_color_distance
                
                if not is_within_tight_distance:
                    if not is_within_max_distance:
                        continue
                    elif not is_similar_color:
                        continue
                
                # if the color and distance are within the prediciton threshold, we consider a match and update
                # FIXME - this assumes constant velocity between frames, which is not always the case
                
                object_id = confirmed_ids[row] # Get the object ID corresponding to this row index
                self._update_confirmed_track(object_id, current_detections_info[col]) # Update confirmed track
                matched_confirmed_rows.add(row) # Mark this confirmed track as matched by adding to set
                matched_detection_cols.add(col) # Mark this detection as matched by adding to set

            self._age_confirmed(matched_confirmed_rows) # Age any confirmed tracks that were not matched this frame
            unmatched_detection_cols -= matched_detection_cols # Remove matched detections from the unmatched set for the next pass

        else:
            self._age_confirmed(set())
            
        filtered_unmatched = set()
        for col in unmatched_detection_cols:
            det_bbox = current_detections_info[col]['bbox']
            overlaps_confirmed = any(
                self._compute_iou(det_bbox, self.tracked_objects[object_id]['bbox']) > self.iou_suppresion_thresh
                for object_id in self.tracked_objects
            )
            if not overlaps_confirmed:
                filtered_unmatched.add(col)
        
        unmatched_detection_cols = filtered_unmatched
            
        # Second pass, match remaining detections against tentative tracks without velocity since they are not confirmed yet
        still_unmatched_cols = filtered_unmatched
        
        if self._tentative_objects and unmatched_detection_cols: # Try to match if tentative tracks exist and unmatched detections remain
            tent_ids       = list(self._tentative_objects.keys())
            tent_centroids = np.array([
                self._tentative_objects[tid]['centroid'] for tid in tent_ids
            ])
            unmatched_col_list = sorted(unmatched_detection_cols)
            unmatched_input    = np.array([
                current_detections_info[c]['centroid'] for c in unmatched_col_list
            ])

            tent_dist  = cdist(tent_centroids, unmatched_input)
            tent_rows  = tent_dist.min(axis=1).argsort()
            tent_cols  = tent_dist.argmin(axis=1)[tent_rows]

            matched_tent_rows = set()
            matched_tent_cols = set()

            for row, col in zip(tent_rows, tent_cols):
                if row in matched_tent_rows or col in matched_tent_cols:
                    continue
                if tent_dist[row, col] > max_distance:
                    continue
                
                
                # If a tentative track is close enough to a detection, consider it a hit
                tent_id      = tent_ids[row]
                original_col = unmatched_col_list[col]

                self._tentative_objects[tent_id] = current_detections_info[original_col]
                self._tentative_hits[tent_id]   += 1 # Update hit count for future promotion

                logging.debug(
                    f"Tentative {tent_id} hit streak: "
                    f"{self._tentative_hits[tent_id]}/{self.hit_streak_required}"
                )

                if self._tentative_hits[tent_id] >= self.hit_streak_required:
                    self._promote_tentative(tent_id) # Promote to confirmed

                matched_tent_rows.add(row)
                matched_tent_cols.add(col)
                still_unmatched_cols.discard(original_col)

            self._age_tentative(matched_tent_rows)

        elif self._tentative_objects:
            # Tentative tracks exist but no detections left to match — discard all
            self._age_tentative(set())

        # Any detections still unmatched after both passes become new tentative tracks
        for col in still_unmatched_cols:
            self._register_tentative(current_detections_info[col])

        return self.tracked_objects
    
    def update_target(self, current_detections_info, target_id, frame_width, frame_height):
        """
        Update target detection only. Call once per frame while tracking.
        Called when tracking for lighter computation overhead rather than determining matches
        of every detection within the frame.
        
        """
        
        max_distance = frame_width * self.max_distance_ratio
        tight_distance = frame_width * self.tight_distance_ratio
        
        if target_id not in self.tracked_objects.keys():
            logging.info(f"Target {target_id} not found")
            return self.tracked_objects
        
        #Single-object predicted centroid computation
        input_centroids = np.array(d['centroid'] for d in current_detections_info)
        cx, cy = self.tracked_objects[target_id]['centroid']
        vx, vy = self.velocities_get(target_id, (0,0))
        
        edge_proximity  = min(cx, frame_width - cx, cy, frame_height - cy) # Distance to nearest edge of the frame
        velocity_weight = float(np.clip(edge_proximity / self.edge_margin, 0.0, 1.0))
        
        px = int(np.clip(cx + vx * velocity_weight, 0, frame_width - 1))  
        py = int(np.clip(cy + vy * velocity_weight, 0, frame_height - 1))
        
        predicted_centroid = np.array([[px, py]])
        
        distance = cdist(predicted_centroid, input_centroids)
        
        # Match testing with each current detection
        
        distance_detections = zip(distance[0], current_detections_info)
        
        for distance, detection in distance_detections:
            if distance > max_distance: # if detection distance is greater than max, skip
                continue
            # If detection distance is within max, it must either be within tight distance or have similar color
            if distance > tight_distance or self._calculate_color_distance(target_id, detection['color']) > self.max_color_distance:
                continue
            
            self._update_confirmed_track(target_id, detection)
        
        return self.tracked_objects[target_id]
             
              
    def _build_predicted_centroids(self, object_ids, frame_width, frame_height):
        """
        Build velocity-predicted centroids for confirmed object IDs.
        Velocity weight is suppressed near frame edges to prevent off-screen
        extrapolation, which destabilises matching for partially visible objects.
        
        :param object_ids: list of confirmed object IDs to build predictions for
        :param frame_width: width of the video frame (for distance normalization)
        :param frame_height: height of the video frame (for distance normalization)
        :return: list of predicted centroid tuples corresponding to the input object IDs
        """
        predicted = []
        for object_id in object_ids:
            cx, cy = self.tracked_objects[object_id]['centroid'] # Centroid of most recent detection for this track
            vx, vy = self.velocities.get(object_id, (0, 0)) # Velocity for this track, default to (0, 0) if not available

            edge_proximity  = min(cx, frame_width - cx, cy, frame_height - cy) # Distance to nearest edge of the frame
            velocity_weight = float(np.clip(edge_proximity / self.edge_margin, 0.0, 1.0)) # Velocity's influence lowers as detections reach the edge to account for partial occlusion.
            
            # Predicted centroid coordinates based on current velocity, clipped to frame bounds
            px = int(np.clip(cx + vx * velocity_weight, 0, frame_width - 1))  
            py = int(np.clip(cy + vy * velocity_weight, 0, frame_height - 1)) 
            predicted.append((px, py))
        return predicted
    
    def _calculate_color_distance(self, object_id, detection_info):
        """
        Calculates and returns the euclidean distance between previous 
        detection and current frame detection color.
        
        :param object_id: ID of the confirmed track to compute color distance
        :param detection_info: detection info dict from current frame corresponding to this track
        :return: euclidean distance between previous color and input color as a floating-point number
        """
        
        input_color = detection_info['color']
        prev_color = self.colors[object_id]
        
        return np.linalg.norm(np.array(input_color) - np.array(prev_color))
    
    def _compute_iou(self, bbox1, bbox2):
        x1 = max(bbox1[0], bbox2[0])
        y1 = max(bbox1[1], bbox2[1])
        x2 = min(bbox1[2], bbox2[2])
        y2 = min(bbox1[3], bbox2[3])
        
        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        if intersection == 0:
            return 0.0
        
        area1 = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1])
        area2 = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])
        union = area1 + area2 - intersection
        return intersection / union if union > 0 else 0.0

    def _update_confirmed_track(self, object_id, detection_info):
        """
        Update a confirmed track with a matched detection and recompute velocity.
        
        :param object_id: ID of the confirmed track to update
        :param detection_info: detection info dict from the current frame that matched this track
        """
        prev_centroid = self.tracked_objects[object_id]['centroid']
        new_centroid  = detection_info['centroid']
        vx = new_centroid[0] - prev_centroid[0] # Per-frame x-direction velocity (pixels/franme)
        vy = new_centroid[1] - prev_centroid[1] # Per-frame y-direction velocity (pixels/frame)
        self.velocities[object_id] = (vx, vy) # Add or update velocity for this track
        self.colors[object_id] = detection_info['color']
        logging.debug(f"Object {object_id} color: {self.colors[object_id]}")

        if vx != 0 or vy != 0:
            logging.debug(f"Object {object_id} velocity: ({vx}, {vy})")

        self.tracked_objects[object_id]    = detection_info
        self.disappeared_frames[object_id] = 0

    def _age_confirmed(self, matched_rows):
        """
        Age all confirmed tracks not matched this frame.
        Decays velocity and deregisters tracks absent too long.
        matched_rows is a set of row indices into the confirmed ID list.
        
        :param matched_rows: set of row indices corresponding to confirmed tracks that were matched this frame
        """
        for row, object_id in enumerate(list(self.tracked_objects.keys())):
            if row in matched_rows:
                continue
            self.disappeared_frames[object_id] += 1
            if self.disappeared_frames[object_id] > self.max_disappeared_frames:
                self._deregister(object_id)
            else:
                # Decay velocity toward zero so prediction doesn't drift during absence
                vx, vy = self.velocities.get(object_id, (0, 0))
                self.velocities[object_id] = (vx * self.velocity_decay, vy * self.velocity_decay)

    def _age_tentative(self, matched_tent_rows):
        """
        Discard any tentative track not matched this frame.
        Tentative tracks get no second chances — one missed frame and they're gone.
        Real objects persist across frames; single-frame ghosts do not.
        
        :param matched_tent_rows: set of row indices corresponding to tentative tracks that were matched this frame
        """
        for row, tent_id in enumerate(list(self._tentative_objects.keys())):
            if row not in matched_tent_rows:
                del self._tentative_objects[tent_id]
                del self._tentative_hits[tent_id]

    def _register_tentative(self, detection_info):
        """
        Add a new tentative track candidate. Uses negative IDs internally.
        
        :param detection_info: detection info dict from the current frame for this new tentative track
        """
        self._tentative_id_seq -= 1
        tid = self._tentative_id_seq
        self._tentative_objects[tid] = detection_info
        self._tentative_hits[tid]    = 1

    def _promote_tentative(self, tent_id):
        """
        Graduate a tentative track to a confirmed track with a stable positive ID.
        
        :param tent_id: ID of the tentative track to promote
        """
        detection_info = self._tentative_objects.pop(tent_id)
        self._tentative_hits.pop(tent_id)
        new_id = self._get_next_id()
        self.tracked_objects[new_id]    = detection_info
        self.disappeared_frames[new_id] = 0 # Start with zero disappeared frames since it's just been confirmed
        self.colors[new_id] = detection_info.get('color', (0,0,0))
       
        logging.debug(f"Track ID {new_id} confirmed after {self.hit_streak_required} consecutive matches.")

    def _get_next_id(self): # Get the next unique ID for a new confirmed track
        new_id = self.next_id_counter
        self.next_id_counter += 1
        return new_id

    def _deregister(self, object_id): # Remove a track that has disappeared for too long
        logging.debug(f"Track ID {object_id} deregistered after {self.disappeared_frames[object_id]} missed frames.")
        del self.tracked_objects[object_id]
        del self.disappeared_frames[object_id]
        self.velocities.pop(object_id, None)
        self.colors.pop(object_id, None)
