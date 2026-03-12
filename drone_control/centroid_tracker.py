import numpy as np
from scipy.spatial.distance import cdist
from collections import OrderedDict

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
