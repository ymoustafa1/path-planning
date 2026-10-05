from __future__ import annotations

import math

from typing import List

from src.models import CarPose, Cone, Path2D


class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Implement ONLY the generatePath function.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes:
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT.
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.

        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.

        Replace the placeholder implementation below with your algorithm.
        """

        car = self.car_pose
        cos_yaw = math.cos(car.yaw)
        sin_yaw = math.sin(car.yaw)
        blue = []
        yellow = []

        for cone in self.cones:
            dx = cone.x - car.x
            dy = cone.y - car.y
            x = dx * cos_yaw + dy * sin_yaw
            y = -dx * sin_yaw + dy * cos_yaw
            if x > 0:
                if cone.color == 1:
                    blue.append((x, y))
                elif cone.color == 0:
                    yellow.append((x, y))

        blue.sort()
        yellow.sort()

        def boundary_y(cones, x):
            if len(cones) == 1 or x <= cones[0][0]:
                return cones[0][1]
            for first, second in zip(cones, cones[1:]):
                if x <= second[0]:
                    break
            x1, y1 = first
            x2, y2 = second
            if abs(x2 - x1) < 0.000001:
                return (y1 + y2) / 2
            return y1 + (y2 - y1) * (x - x1) / (x2 - x1)

        distances = sorted({x for x, y in blue + yellow})
        distances.append(max(8.0, distances[-1] + 1.0) if distances else 8.0)
        waypoints = [(0.0, 0.0)]

        for x in distances:
            if blue and yellow:
                y = (boundary_y(blue, x) + boundary_y(yellow, x)) / 2
            elif blue:
                y = boundary_y(blue, x) - 1.0
            elif yellow:
                y = boundary_y(yellow, x) + 1.0
            else:
                # Default: produce a short straight-ahead path from the current pose.
                # delete/replace this with your own algorithm.
                y = 0.0
            waypoints.append((x, y))

        path = [(car.x, car.y)]
        remaining = 8.0

        for (x1, y1), (x2, y2) in zip(waypoints, waypoints[1:]):
            length = math.hypot(x2 - x1, y2 - y1)
            distance = min(length, remaining)
            steps = math.ceil(distance / 0.5)
            for i in range(1, steps + 1):
                fraction = distance * i / steps / length
                x = x1 + (x2 - x1) * fraction
                y = y1 + (y2 - y1) * fraction
                path.append((car.x + x * cos_yaw - y * sin_yaw,
                             car.y + x * sin_yaw + y * cos_yaw))
            remaining -= distance
            if remaining < 0.000001:
                break

        return path
