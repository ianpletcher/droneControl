import numpy as np

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