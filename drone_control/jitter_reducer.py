import math

# -----------------------------------------------------------------------------------------------
# Tricking Smoother to Reduce Jitter (not implemented in main)z
# -----------------------------------------------------------------------------------------------
# Implements scaling and deadzone to reduze the movement when a target is relatively centered.
# Helps with flight stability, also reduces the num of commands required, saving a lot of overhead.
class JitterReducer:
    def __init__(self, dead_zone_radius=10, smoothing_factor=0.8):
        # Initialize the jitter reducer.
        self.dead_zone_radius = dead_zone_radius
        self.smoothing_factor = smoothing_factor
        self.previous_position = None

    def is_in_dead_zone(self, x, y, center_x, center_y):
        # Check target is within dead zone.
        distance = math.sqrt((x - center_x) ** 2 + (y - center_y) ** 2)
        return distance < self.dead_zone_radius

    def smooth_position(self, current_position):
        # Smooth the position using an exponential moving average.

        if self.previous_position is None:
            self.previous_position = current_position
            return current_position

        smoothed_x = (
            self.smoothing_factor * current_position[0]
            + (1 - self.smoothing_factor) * self.previous_position[0]
        )
        smoothed_y = (
            self.smoothing_factor * current_position[1]
            + (1 - self.smoothing_factor) * self.previous_position[1]
        )

        self.previous_position = (smoothed_x, smoothed_y)
        return smoothed_x, smoothed_y

    def reduce_jitter(self, x, y, center_x, center_y):
        # Reduce jitter for the given position.

        if self.is_in_dead_zone(x, y, center_x, center_y):
            # If in the dead zone, return the center position
            return center_x, center_y
        else:
            # Smooth the position
            return self.smooth_position((x, y))