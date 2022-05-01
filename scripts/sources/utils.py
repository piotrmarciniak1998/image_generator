import cv2
import os
import numpy as np
from random import uniform
from math import sqrt
from cv_bridge import CvBridge


def position_on_item(item, colliders=[], target_pose=None, camera_pose=None, tolerance=0.9, is_obstructor=False):
    x_min, x_max, y_min, y_max, height = item.get_bounding_box_of_normalized_item()
    a = x_max * tolerance
    b = y_max * tolerance

    if is_obstructor:
        x, y = get_average_x_y(target_pose, camera_pose)
        x += uniform(-item.scale * 0.02, item.scale * 0.02)
        y += uniform(-item.scale * 0.02, item.scale * 0.02)
        return x, y, height

    else:
        while True:
            x = uniform(x_min, x_max)
            y = uniform(y_min, y_max)

            collision = False
            if (x * x) / (a * a) + (y * y) / (b * b) < 1:
                for collider in colliders:
                    collider_x, collider_y, _ = collider.get_pose()
                    new_pose_radius = (x - collider_x) * (x - collider_x) + (y - collider_y) * (y - collider_y)
                    if collider.x_span > collider.y_span:
                        if new_pose_radius < collider.x_span * collider.x_span * 2.5:
                            collision = True
                            break
                    else:
                        if new_pose_radius < collider.y_span * collider.y_span * 2.5:
                            collision = True
                            break

                if target_pose and camera_pose:
                    slope = (target_pose[1] - camera_pose[1]) / (target_pose[0] - camera_pose[0])
                    translation = target_pose[1] - slope * target_pose[0]

                    line_a = -slope
                    line_b = 1
                    line_c = -translation

                    distance_to_line = abs(line_a * x + line_b * y + line_c) / sqrt(line_a * line_a + line_b * line_b)

                    if distance_to_line < item.scale * 0.05:
                        collision = True

                if not collision:
                    return x, y, height


def get_average_x_y(pos1, pos2):
    x = (pos1[0] + pos2[0]) / 2
    y = (pos1[1] + pos2[1]) / 2
    return x, y


def calculate_difference(img_1_msg, img_2_msg):
    bridge = CvBridge()
    img_1_msg.encoding = "32FC1"
    img_2_msg.encoding = "32FC1"
    img_1 = bridge.imgmsg_to_cv2(img_1_msg, desired_encoding='passthrough')
    img_2 = bridge.imgmsg_to_cv2(img_2_msg, desired_encoding='passthrough')
    diff = np.sum(img_1 != img_2)
    return diff


def calculate_occlusion(msg_empty, msg_obstructor, msg_target, msg_target_obstructor):
    tar_pix = calculate_difference(msg_target, msg_empty)
    obs_pix = calculate_difference(msg_target_obstructor, msg_obstructor)
    occlusion = abs(int(round((tar_pix - obs_pix) / tar_pix * 100, 0)))
    return occlusion


def add_text(index, dirname, occlusion="", target_pos="", photo_kinds=None):
    if photo_kinds is None:
        photo_kinds = ['depth_u', 'depth_o', 'rgb_u', 'rgb_o']
    text = ""
    if occlusion != "":
        text += f"_{occlusion}"
    if target_pos != "":
        text += f"_{target_pos}"
    for kind in photo_kinds:
        try:
            os.rename(f"{dirname}/{index}_{kind}.png",
                      f"{dirname}/{index}_{kind}{text}.png")
        except FileNotFoundError:
            pass


def calculate_target_pos(msg_empty, msg_target):
    bridge = CvBridge()
    msg_empty.encoding = "32FC1"
    msg_target.encoding = "32FC1"
    img_empty = bridge.imgmsg_to_cv2(msg_empty, desired_encoding='passthrough')
    img_target = bridge.imgmsg_to_cv2(msg_target, desired_encoding='passthrough')
    img_empty = np.array(((img_empty - 0.05) / 3.95) * 255, dtype=np.uint8)
    img_target = np.array(((img_target - 0.05) / 3.95) * 255, dtype=np.uint8)
    diff = cv2.absdiff(img_target, img_empty)
    _, th = cv2.threshold(diff, 1, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    biggest_contour = contours[0]
    for contour in contours:
        if cv2.contourArea(biggest_contour) < cv2.contourArea(contour):
            biggest_contour = contour
    m = cv2.moments(biggest_contour)
    cx = int(m['m10'] / m['m00'])
    cy = int(m['m01'] / m['m00'])
    return cx, cy


def display_metrics(current_iteration, target_iterations, time_start, time_now):
    average_time = (time_now - time_start) / (current_iteration + 0.000001)

    completion_percentage = round(current_iteration / target_iterations * 100, 1)

    running_time = int(round(time_now - time_start, 0))
    running_seconds = running_time % 60
    running_minutes = (running_time // 60) % 60
    running_hours = running_time // 3600
    if running_hours > 0:
        running_time_text = f"{running_hours}h, {running_minutes}m, {running_seconds}s"
    elif running_minutes > 0:
        running_time_text = f"{running_minutes}m, {running_seconds}s"
    else:
        running_time_text = f"{running_seconds}s"

    estimated_time = int(round(average_time * (target_iterations - current_iteration), 0))
    estimated_seconds = estimated_time % 60
    estimated_minutes = (estimated_time // 60) % 60
    estimated_hours = estimated_time // 3600
    if estimated_hours > 0:
        estimated_time_text = f"{estimated_hours}h, {estimated_minutes}m, {estimated_seconds}s"
    elif estimated_minutes > 0:
        estimated_time_text = f"{estimated_minutes}m, {estimated_seconds}s"
    else:
        estimated_time_text = f"{estimated_seconds}s"

    return f"\t{current_iteration}/{target_iterations}, " \
           f"\t{completion_percentage}%, " \
           f"\tStart: {running_time_text}, " \
           f"\tLeft: {estimated_time_text}, " \
           f"\tAvg: {round(average_time, 3)}s"
