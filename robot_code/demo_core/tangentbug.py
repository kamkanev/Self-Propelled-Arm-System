from __future__ import print_function

import cv2
import numpy as np


class TangentPlan(object):
    def __init__(self, action, heading_error=0.0, path_clear=False, candidates=None, profile=None, reason=None):
        self.action = action
        self.heading_error = float(heading_error)
        self.path_clear = bool(path_clear)
        self.candidates = candidates or []
        self.profile = profile
        self.reason = reason

    def as_dict(self):
        return {
            "action": self.action,
            "heading_error": self.heading_error,
            "path_clear": self.path_clear,
            "candidates": self.candidates,
            "reason": self.reason,
        }


class DepthTangentBugPlanner(object):
    """TangentBug-inspired local planner for a monocular depth field.

    The original TangentBug assumes metric range sensing and robot pose. This
    adapter builds a one-dimensional local tangent graph from horizontal depth
    discontinuities. It is a reactive demo planner and does not claim the
    original algorithm's global convergence guarantee.
    """

    def __init__(self, settings):
        self.settings = settings

    def _column_profile(self, depth_map):
        depth = np.asarray(depth_map, dtype=np.float32)
        if depth.ndim > 2:
            depth = depth[:, :, 0]
        height, width = depth.shape[:2]
        y1 = max(0, min(height - 1, int(height * float(self.settings.get("profile_row_start", 0.45)))))
        y2 = max(y1 + 1, min(height, int(height * float(self.settings.get("profile_row_end", 0.9)))))
        columns = max(8, int(self.settings.get("profile_columns", 80)))
        edges = np.linspace(0, width, columns + 1).astype(np.int32)
        profile = np.full(columns, np.inf, dtype=np.float32)
        band = depth[y1:y2]
        for index in range(columns):
            region = band[:, edges[index]:edges[index + 1]]
            values = region[np.isfinite(region) & (region > 0.0)]
            if values.size:
                profile[index] = float(np.median(values))
        window = max(1, int(self.settings.get("smoothing_window", 5)))
        if window > 1:
            kernel = np.ones(window, dtype=np.float32) / float(window)
            finite_cap = float(self.settings.get("clear_depth", 1.6)) * 2.0
            smooth_input = np.where(np.isfinite(profile), profile, finite_cap)
            profile = np.convolve(smooth_input, kernel, mode="same").astype(np.float32)
        return profile

    @staticmethod
    def _segments(mask):
        segments = []
        start = None
        for index, value in enumerate(mask):
            if value and start is None:
                start = index
            if start is not None and (not value or index == len(mask) - 1):
                end = index if value and index == len(mask) - 1 else index - 1
                segments.append((start, end))
                start = None
        return segments

    def plan(self, depth_map, target_error_x=0.0):
        if depth_map is None:
            return TangentPlan("blocked", reason="depth unavailable")
        profile = self._column_profile(depth_map)
        count = len(profile)
        target_index = int(round((0.5 + float(target_error_x)) * float(count - 1)))
        target_index = max(0, min(count - 1, target_index))
        obstacle_depth = float(self.settings.get("obstacle_depth", 1.2))
        clear_depth = float(self.settings.get("clear_depth", 1.6))
        minimum_gap = max(1, int(self.settings.get("minimum_gap_columns", 6)))
        half_corridor = max(1, minimum_gap // 2)
        direct_start = max(0, target_index - half_corridor)
        direct_end = min(count, target_index + half_corridor + 1)
        direct_clear = bool(np.all(profile[direct_start:direct_end] >= clear_depth))
        if direct_clear:
            return TangentPlan(
                "forward",
                heading_error=float(target_error_x),
                path_clear=True,
                profile=profile,
                reason="direct corridor clear",
            )

        free = profile >= obstacle_depth
        segments = [segment for segment in self._segments(free) if segment[1] - segment[0] + 1 >= minimum_gap]
        if not segments:
            return TangentPlan("blocked", profile=profile, reason="no traversable depth gap")

        discontinuity = float(self.settings.get("discontinuity_threshold", 0.3))
        heading_weight = float(self.settings.get("heading_weight", 1.0))
        clearance_weight = float(self.settings.get("clearance_weight", 0.35))
        candidates = []
        for start, end in segments:
            indices = {start, end, int(round((start + end) / 2.0))}
            for index in sorted(indices):
                heading_error = (float(index) / float(max(1, count - 1))) - 0.5
                clearance = float(profile[index])
                tangent_bonus = 0.0
                if index > 0 and abs(float(profile[index]) - float(profile[index - 1])) >= discontinuity:
                    tangent_bonus = -0.1
                if index + 1 < count and abs(float(profile[index + 1]) - float(profile[index])) >= discontinuity:
                    tangent_bonus = -0.1
                cost = (
                    heading_weight * abs(index - target_index) / float(count)
                    + clearance_weight / max(clearance, 0.05)
                    + tangent_bonus
                )
                candidates.append({
                    "index": int(index),
                    "heading_error": float(heading_error),
                    "clearance": clearance,
                    "cost": float(cost),
                })
        chosen = min(candidates, key=lambda item: item["cost"])
        heading_error = float(chosen["heading_error"])
        if abs(heading_error) <= 0.08:
            action = "forward"
        else:
            action = "right" if heading_error > 0.0 else "left"
        return TangentPlan(
            action,
            heading_error=heading_error,
            path_clear=False,
            candidates=candidates,
            profile=profile,
            reason="local tangent candidate",
        )

    def draw_debug(self, frame, plan):
        canvas = np.asarray(frame).copy()
        height, width = canvas.shape[:2]
        profile = plan.profile
        if profile is None or len(profile) == 0:
            cv2.putText(canvas, "TangentBug: {}".format(plan.reason), (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
            return canvas
        max_depth = max(float(self.settings.get("clear_depth", 1.6)) * 1.5, 0.1)
        points = []
        for index, value in enumerate(profile):
            x = int(round(float(index) * float(width - 1) / float(max(1, len(profile) - 1))))
            normalized = min(max(float(value), 0.0), max_depth) / max_depth
            y = int(round(float(height - 1) - normalized * float(height) * 0.35))
            points.append([x, y])
            color = (0, 255, 0) if value >= float(self.settings.get("obstacle_depth", 1.2)) else (0, 0, 255)
            cv2.line(canvas, (x, height - 1), (x, y), color, 1)
        if len(points) > 1:
            cv2.polylines(canvas, [np.asarray(points, dtype=np.int32)], False, (255, 255, 0), 1)
        for candidate in plan.candidates:
            x = int(round(float(candidate["index"]) * float(width - 1) / float(max(1, len(profile) - 1))))
            cv2.circle(canvas, (x, height - 8), 4, (255, 0, 255), -1)
        cv2.putText(
            canvas,
            "TangentBug action={} clear={} err={:.3f}".format(plan.action, plan.path_clear, plan.heading_error),
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
        )
        return canvas
