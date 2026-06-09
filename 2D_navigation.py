import cv2
import numpy as np
from rplidar import RPLidar
import math
import tkinter as tk

PORT_NAME = "COM8"

MAP_SIZE = 1200
MAP_RESOLUTION = 20

SHOW_TRAILS = True

MIN_DISTANCE = 150
MAX_DISTANCE = 12000

CLUSTER_DISTANCE = 200
MIN_CLUSTER_POINTS = 4
WARNING_DISTANCE = 1000

WINDOW_NAME = "RPLIDAR 2D Mapping + Obstacle Detection"

def get_screen_resolution():

    root = tk.Tk()
    root.withdraw()

    width = root.winfo_screenwidth()
    height = root.winfo_screenheight()

    root.destroy()

    return width, height

occupancy_map = np.zeros(
    (MAP_SIZE, MAP_SIZE),
    dtype=np.uint8
)

CENTER_X = MAP_SIZE // 2
CENTER_Y = MAP_SIZE // 2

def polar_to_map(angle_deg, distance_mm):

    angle_rad = math.radians(angle_deg)

    x_mm = distance_mm * math.cos(angle_rad)
    y_mm = distance_mm * math.sin(angle_rad)

    px = int(CENTER_X + x_mm / MAP_RESOLUTION)
    py = int(CENTER_Y - y_mm / MAP_RESOLUTION)

    return px, py

def draw_grid(img):

    h, w = img.shape[:2]

    grid_spacing = 50

    for x in range(0, w, grid_spacing):
        cv2.line(img, (x, 0), (x, h), (40, 40, 40), 1)

    for y in range(0, h, grid_spacing):
        cv2.line(img, (0, y), (w, y), (40, 40, 40), 1)

    cv2.line(
        img,
        (CENTER_X, 0),
        (CENTER_X, h),
        (100, 100, 100),
        1
    )

    cv2.line(
        img,
        (0, CENTER_Y),
        (w, CENTER_Y),
        (100, 100, 100),
        1
    )

def detect_obstacles(scan):

    clusters = []
    current_cluster = []

    valid_points = []

    for quality, angle, distance in scan:

        if distance < MIN_DISTANCE:
            continue

        if distance > MAX_DISTANCE:
            continue

        x = distance * math.cos(
            math.radians(angle)
        )

        y = distance * math.sin(
            math.radians(angle)
        )

        valid_points.append((x, y))

    if len(valid_points) == 0:
        return []

    for point in valid_points:

        if len(current_cluster) == 0:
            current_cluster.append(point)
            continue

        last_x, last_y = current_cluster[-1]

        d = math.sqrt(
            (point[0] - last_x) ** 2 +
            (point[1] - last_y) ** 2
        )

        if d < CLUSTER_DISTANCE:

            current_cluster.append(point)

        else:

            if len(current_cluster) >= MIN_CLUSTER_POINTS:
                clusters.append(current_cluster)

            current_cluster = [point]

    if len(current_cluster) >= MIN_CLUSTER_POINTS:
        clusters.append(current_cluster)

    obstacles = []

    for cluster in clusters:

        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]

        center_x = np.mean(xs)
        center_y = np.mean(ys)

        width = max(xs) - min(xs)
        height = max(ys) - min(ys)

        distance = math.sqrt(
            center_x ** 2 +
            center_y ** 2
        )

        obstacles.append({
            "x": center_x,
            "y": center_y,
            "distance": distance,
            "width": width,
            "height": height,
            "points": len(cluster)
        })

    return obstacles

def main():

    screen_width, screen_height = get_screen_resolution()

    cv2.namedWindow(
        WINDOW_NAME,
        cv2.WINDOW_NORMAL
    )

    cv2.setWindowProperty(
        WINDOW_NAME,
        cv2.WND_PROP_FULLSCREEN,
        cv2.WINDOW_FULLSCREEN
    )

    print("Connecting to RPLIDAR...")

    lidar = RPLidar(PORT_NAME)

    try:

        print("RPLIDAR INFO:")
        print(lidar.get_info())

        print("RPLIDAR HEALTH:")
        print(lidar.get_health())

        scan_count = 0

        for scan in lidar.iter_scans(max_buf_meas=1500):

            scan_count += 1

            if not SHOW_TRAILS:
                occupancy_map[:] = 0

            for quality, angle, distance in scan:

                if distance < MIN_DISTANCE:
                    continue

                if distance > MAX_DISTANCE:
                    continue

                px, py = polar_to_map(
                    angle,
                    distance
                )

                if (
                    0 <= px < MAP_SIZE and
                    0 <= py < MAP_SIZE
                ):
                    occupancy_map[py, px] = 255

            obstacles = detect_obstacles(scan)

            display = cv2.cvtColor(
                occupancy_map,
                cv2.COLOR_GRAY2BGR
            )

            draw_grid(display)

            cv2.circle(
                display,
                (CENTER_X, CENTER_Y),
                8,
                (0, 0, 255),
                -1
            )

            nearest_distance = 999999

            for obs in obstacles:

                px = int(
                    CENTER_X +
                    obs["x"] / MAP_RESOLUTION
                )

                py = int(
                    CENTER_Y -
                    obs["y"] / MAP_RESOLUTION
                )

                distance = int(
                    obs["distance"]
                )

                nearest_distance = min(
                    nearest_distance,
                    distance
                )

                radius = max(
                    10,
                    int(
                        max(
                            obs["width"],
                            obs["height"]
                        ) /
                        MAP_RESOLUTION / 2
                    )
                )

                cv2.circle(
                    display,
                    (px, py),
                    radius,
                    (0, 255, 255),
                    2
                )

                cv2.putText(
                    display,
                    f"{distance} mm",
                    (px + 15, py),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1
                )

            cv2.putText(
                display,
                f"Scans: {scan_count}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            cv2.putText(
                display,
                f"Obstacles: {len(obstacles)}",
                (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 0),
                2
            )

            cv2.putText(
                display,
                "ESC=Exit  C=Clear",
                (10, MAP_SIZE - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            if nearest_distance < WARNING_DISTANCE:

                cv2.putText(
                    display,
                    "WARNING: OBSTACLE CLOSE!",
                    (250, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    3
                )

            fullscreen_display = cv2.resize(
                display,
                (
                    screen_width,
                    screen_height
                ),
                interpolation=cv2.INTER_NEAREST
            )

            cv2.imshow(
                WINDOW_NAME,
                fullscreen_display
            )

            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                break

            elif key == ord('c'):
                occupancy_map[:] = 0

    except KeyboardInterrupt:

        print("Stopped by user")

    finally:

        print("Saving map...")

        cv2.imwrite(
            "rplidar_obstacle_map.png",
            occupancy_map
        )

        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except:
            pass

        cv2.destroyAllWindows()

        print(
            "Saved: rplidar_obstacle_map.png"
        )

if __name__ == "__main__":
    main()
