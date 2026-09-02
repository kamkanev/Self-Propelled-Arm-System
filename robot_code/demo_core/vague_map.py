from __future__ import print_function

import math
import time


def normalize_heading(angle_rad):
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


class Point2D(object):
    def __init__(self, x_m=0.0, y_m=0.0):
        self.x_m = float(x_m)
        self.y_m = float(y_m)

    @classmethod
    def from_mapping(cls, value):
        return cls(value.get("x_m", 0.0), value.get("y_m", 0.0))

    def distance_to(self, other):
        return math.hypot(self.x_m - other.x_m, self.y_m - other.y_m)

    def as_dict(self):
        return {"x_m": self.x_m, "y_m": self.y_m}


class Pose2D(Point2D):
    def __init__(self, x_m=0.0, y_m=0.0, heading_rad=0.0):
        Point2D.__init__(self, x_m, y_m)
        self.heading_rad = normalize_heading(heading_rad)

    @classmethod
    def from_mapping(cls, value):
        return cls(
            value.get("x_m", 0.0),
            value.get("y_m", 0.0),
            value.get("heading_rad", 0.0),
        )

    def copy(self):
        return Pose2D(self.x_m, self.y_m, self.heading_rad)

    def as_dict(self):
        result = Point2D.as_dict(self)
        result["heading_rad"] = self.heading_rad
        return result


class MappedCan(object):
    def __init__(self, can_id, position, confidence, timestamp=None):
        self.can_id = int(can_id)
        self.position = position
        self.confidence = float(confidence)
        self.observations = 1
        self.last_seen_at = float(timestamp if timestamp is not None else time.time())

    def merge(self, position, confidence, timestamp=None):
        count = float(self.observations)
        self.position.x_m = (self.position.x_m * count + position.x_m) / (count + 1.0)
        self.position.y_m = (self.position.y_m * count + position.y_m) / (count + 1.0)
        self.observations += 1
        self.confidence = max(self.confidence, float(confidence))
        self.last_seen_at = float(timestamp if timestamp is not None else time.time())

    def as_dict(self):
        return {
            "can_id": self.can_id,
            "position": self.position.as_dict(),
            "confidence": self.confidence,
            "observations": self.observations,
            "last_seen_at": self.last_seen_at,
        }


class VagueMap(object):
    """Runtime-only spatial memory using command-based odometry."""

    def __init__(self, settings):
        self.settings = settings
        self.bounds = dict(settings["bounds_m"])
        self.initial_pose = Pose2D.from_mapping(settings["initial_pose"])
        self.robot_pose = self.initial_pose.copy()
        self.bin_marker_position = Point2D.from_mapping(settings["bin_marker_position"])
        self.bin_docking_pose = Pose2D.from_mapping(settings["bin_docking_pose"])
        self.bin_side_docking_pose = Pose2D.from_mapping(
            settings.get("bin_side_docking_pose", settings["bin_docking_pose"])
        )
        self.patrol_waypoints = [Point2D.from_mapping(value) for value in settings["patrol_waypoints"]]
        self.patrol_index = 0
        self.patrol_complete = len(self.patrol_waypoints) == 0
        self.initial_scan_complete = False
        self.known_cans = {}
        self.selected_can_id = None
        self._next_can_id = 1

    def contains(self, point):
        return (
            float(self.bounds["min_x"]) <= point.x_m <= float(self.bounds["max_x"])
            and float(self.bounds["min_y"]) <= point.y_m <= float(self.bounds["max_y"])
        )

    def set_robot_pose(self, pose, reason="manual"):
        self.robot_pose = pose.copy()
        print("[map] pose reset reason={} pose={}".format(reason, self.robot_pose.as_dict()))

    def estimate_can_position(self, observation, depth_value, image_height):
        if not observation or not observation.get("found") or depth_value is None:
            return None
        center_y_norm = float(observation["center_y"]) / float(max(1, int(image_height)))
        minimum_y = float(self.settings.get("incidental_can_min_center_y_norm", 0.45))
        if center_y_norm < minimum_y:
            print("[map] can rejected reason=upper_frame center_y_norm={:.3f}".format(center_y_norm))
            return None
        distance_m = float(depth_value) * float(self.settings.get("depth_to_distance_scale_m_per_unit", 1.0))
        if not math.isfinite(distance_m) or distance_m <= 0.0:
            print("[map] can rejected reason=invalid_depth value={}".format(depth_value))
            return None
        bearing_offset = -float(observation["error_x"]) * float(self.settings["camera_horizontal_fov_rad"])
        bearing = self.robot_pose.heading_rad + bearing_offset
        return Point2D(
            self.robot_pose.x_m + distance_m * math.cos(bearing),
            self.robot_pose.y_m + distance_m * math.sin(bearing),
        )

    def remember_can(self, position, confidence, timestamp=None):
        if not self.contains(position):
            print("[map] can rejected reason=out_of_bounds position={}".format(position.as_dict()))
            return None
        merge_radius = float(self.settings.get("can_merge_radius_m", 0.3))
        nearest = None
        nearest_distance = None
        for mapped_can in self.known_cans.values():
            distance = mapped_can.position.distance_to(position)
            if nearest_distance is None or distance < nearest_distance:
                nearest = mapped_can
                nearest_distance = distance
        if nearest is not None and nearest_distance <= merge_radius:
            nearest.merge(position, confidence, timestamp)
            print("[map] can merged id={} position={} observations={}".format(
                nearest.can_id, nearest.position.as_dict(), nearest.observations
            ))
            return nearest
        mapped_can = MappedCan(self._next_can_id, position, confidence, timestamp)
        self._next_can_id += 1
        self.known_cans[mapped_can.can_id] = mapped_can
        print("[map] can added id={} position={}".format(mapped_can.can_id, position.as_dict()))
        return mapped_can

    def nearest_can(self):
        if not self.known_cans:
            self.selected_can_id = None
            return None
        mapped_can = min(
            self.known_cans.values(),
            key=lambda item: self.robot_pose.distance_to(item.position),
        )
        self.selected_can_id = mapped_can.can_id
        print("[map] selected can id={} position={}".format(mapped_can.can_id, mapped_can.position.as_dict()))
        return mapped_can

    def remove_can(self, can_id, reason="removed"):
        mapped_can = self.known_cans.pop(int(can_id), None)
        if mapped_can is not None:
            print("[map] can removed id={} reason={}".format(mapped_can.can_id, reason))
        if self.selected_can_id == int(can_id):
            self.selected_can_id = None
        return mapped_can

    def remove_cans_near_robot(self):
        radius = float(self.settings.get("pickup_clear_radius_m", 0.3))
        remove_ids = [
            can_id
            for can_id, mapped_can in self.known_cans.items()
            if self.robot_pose.distance_to(mapped_can.position) <= radius
        ]
        for can_id in remove_ids:
            self.remove_can(can_id, "pickup_clear_radius")
        if self.selected_can_id is not None:
            self.remove_can(self.selected_can_id, "selected_can_picked")

    def current_patrol_waypoint(self):
        if self.patrol_complete or self.patrol_index >= len(self.patrol_waypoints):
            return None
        return self.patrol_waypoints[self.patrol_index]

    def advance_patrol(self):
        if not self.patrol_complete:
            self.patrol_index += 1
            self.patrol_complete = self.patrol_index >= len(self.patrol_waypoints)
        print("[map] patrol index={} complete={}".format(self.patrol_index, self.patrol_complete))
        return self.current_patrol_waypoint()

    def snapshot(self):
        return {
            "robot_pose": self.robot_pose.as_dict(),
            "bin_marker_position": self.bin_marker_position.as_dict(),
            "bin_docking_pose": self.bin_docking_pose.as_dict(),
            "bin_side_docking_pose": self.bin_side_docking_pose.as_dict(),
            "patrol_index": self.patrol_index,
            "patrol_complete": self.patrol_complete,
            "initial_scan_complete": self.initial_scan_complete,
            "selected_can_id": self.selected_can_id,
            "known_cans": [item.as_dict() for item in sorted(self.known_cans.values(), key=lambda value: value.can_id)],
        }


class CommandOdometry(object):
    def __init__(self, vague_map, settings):
        self.vague_map = vague_map
        self.settings = settings

    def record_motion(self, direction, speed, effective_seconds):
        direction = str(direction)
        speed = abs(float(speed))
        effective_seconds = max(0.0, float(effective_seconds))
        pose = self.vague_map.robot_pose
        if direction in ("forward", "backward"):
            sign = 1.0 if direction == "forward" else -1.0
            distance = (
                sign
                * speed
                * effective_seconds
                * float(self.settings["linear_meters_per_speed_second"])
                * float(self.settings.get("linear_slip_factor", 1.0))
            )
            pose.x_m += distance * math.cos(pose.heading_rad)
            pose.y_m += distance * math.sin(pose.heading_rad)
        elif direction in ("left", "right"):
            sign = 1.0 if direction == "left" else -1.0
            pose.heading_rad = normalize_heading(
                pose.heading_rad
                + sign * speed * effective_seconds * float(self.settings["angular_radians_per_speed_second"])
            )
        else:
            raise ValueError("unknown odometry direction: {}".format(direction))
        print("[map] motion direction={} speed={} effective_seconds={} pose={}".format(
            direction, speed, effective_seconds, pose.as_dict()
        ))
        return pose.copy()


class VagueMapNavigator(object):
    """Coarse point navigation. Visual target handling remains outside this class."""

    def __init__(self, vague_map, base, settings):
        self.vague_map = vague_map
        self.base = base
        self.settings = settings

    def step_toward(self, destination, label, arrival_tolerance_m=None):
        turn_label = "map_turn_{}".format(label)
        turning = hasattr(self.base, "motion_active") and self.base.motion_active(turn_label)
        dry_run_step = self.settings.get(
            "turn_pulse_seconds" if turning else "forward_pulse_seconds",
            0.0,
        )
        self.base.update_motion_odometry(float(dry_run_step))
        pose = self.vague_map.robot_pose
        distance = pose.distance_to(destination)
        tolerance = float(
            arrival_tolerance_m
            if arrival_tolerance_m is not None
            else self.settings.get("arrival_tolerance_m", 0.2)
        )
        if distance <= tolerance:
            self.base.stop()
            print("[map] destination reached label={} distance_m={:.3f}".format(label, distance))
            return True
        desired_heading = math.atan2(destination.y_m - pose.y_m, destination.x_m - pose.x_m)
        heading_error = normalize_heading(desired_heading - pose.heading_rad)
        print("[map] navigate label={} distance_m={:.3f} heading_error_rad={:.3f}".format(
            label, distance, heading_error
        ))
        if abs(heading_error) > float(self.settings["heading_tolerance_rad"]):
            self.base.start_motion(
                "left" if heading_error > 0.0 else "right",
                float(self.settings["turn_speed"]),
                turn_label,
            )
        else:
            self.base.start_motion(
                "forward", float(self.settings["forward_speed"]), "map_forward_{}".format(label)
            )
        return False

    def patrol_step(self):
        waypoint = self.vague_map.current_patrol_waypoint()
        if waypoint is None:
            return True
        reached = self.step_toward(
            waypoint,
            "patrol_{}".format(self.vague_map.patrol_index),
            float(self.settings.get("waypoint_tolerance_m", 0.15)),
        )
        if reached:
            self.vague_map.advance_patrol()
        return self.vague_map.patrol_complete
