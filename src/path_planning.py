import math

from src.models import CarPose, Cone, Path2D


class PathPlanning:
    def __init__(self, car_pose: CarPose, cones: list[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
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
