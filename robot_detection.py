import threading
from time import time
from math import atan2, cos, sin, hypot

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst

import hailo
from serial import Serial

from hailo_apps.python.core.gstreamer.gstreamer_app import (
    app_callback_class,
)
from hailo_apps.python.pipeline_apps.detection.detection_pipeline import (
    GStreamerDetectionApp,
)


class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()

        # Pico serial connection
       # self.pico_msngr = Serial(
       #     port="/dev/ttyACM0",
       #     baudrate=115200,
       #     timeout=0.01,
       # )

               # Pico serial disabled for camera/Hailo test
        self.pico_msngr = None
        print("Pico serial disabled for camera/Hailo test")

	# Robot state
        self.is_goal_reached = True

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.targ_lin_vel = 0.0
        self.targ_ang_vel = 0.0

        self.motion_data = {
            "meas_lin_vel": 0.0,
            "fuse_ang_vel": 0.0,
        }

        self.last_ts = time()

        # Start Pico communication thread
        #self.pico_thread = threading.Thread(
        #    target=self.process_pico_msgs,
        #    daemon=True,
        #)
        #self.pico_thread.start()

    def process_pico_msgs(self):
        last_ts = time()

        while self.pico_msngr is not None:
            curr_ts = time()
            dt = curr_ts - last_ts

            if dt >= 0.04:

                if not self.is_goal_reached:
                    self.compute_target_velocity()

                msg_to_pico = (
                    f"{self.targ_lin_vel:.3f},"
                    f"{self.targ_ang_vel:.3f}\n"
                )

                self.pico_msngr.write(msg_to_pico.encode("utf-8"))

                last_ts = curr_ts

                # Integrate measured motion
                self.x += (
                    self.motion_data["meas_lin_vel"]
                    * cos(self.theta)
                    * dt
                )

                self.y += (
                    self.motion_data["meas_lin_vel"]
                    * sin(self.theta)
                    * dt
                )

                self.theta += (
                    self.motion_data["fuse_ang_vel"] * dt
                )

                self.theta = atan2(
                    sin(self.theta),
                    cos(self.theta),
                )

            # Read Pico telemetry
            if self.pico_msngr.inWaiting() > 0:
                msg_from_pico = (
                    self.pico_msngr
                    .readline()
                    .decode("utf-8", "ignore")
                    .strip()
                )

                if msg_from_pico:
                    data_strings = msg_from_pico.split(",")

                    try:
                        self.motion_data.update(
                            zip(
                                self.motion_data.keys(),
                                map(float, data_strings),
                            )
                        )
                    except ValueError:
                        pass

    def compute_target_velocity(
        self,
        kp_v=0.5,
        kp_w=0.5,
        max_v=0.3,
        max_w=0.6,
        distance_tolerance=0.05,
    ):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        distance_error = hypot(dx, dy)

        if distance_error < distance_tolerance:
            self.is_goal_reached = True
            self.targ_lin_vel = 0.0
            self.targ_ang_vel = 0.0

        else:
            self.is_goal_reached = False

            target_heading = atan2(dy, dx)

            heading_error = target_heading - self.theta

            heading_error = atan2(
                sin(heading_error),
                cos(heading_error),
            )

            cmd_w = kp_w * heading_error

            direction_alignment = max(
                0.0,
                cos(heading_error),
            )

            cmd_v = (
                kp_v
                * distance_error
                * direction_alignment
            )

            self.targ_lin_vel = max(
                min(cmd_v, max_v),
                -max_v,
            )

            self.targ_ang_vel = max(
                min(cmd_w, max_w),
                -max_w,
            )

    def set_goal(self, goal_x, goal_y):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.is_goal_reached = False


def app_callback(element, buffer, user_data):

    # Original behavior: continuously command the goal.
    # user_data.set_goal(1.0, -0.5)

    print(user_data.motion_data)

    if buffer is None:
        return

    string_to_print = (
        f"Frame count: {user_data.get_count()}\n"
    )

    roi = hailo.get_roi_from_buffer(buffer)

    detections = roi.get_objects_typed(
        hailo.HAILO_DETECTION
    )

    for detection in detections:
        label = detection.get_label()
        confidence = detection.get_confidence()

        string_to_print += (
            f"Detection: {label} "
            f"Confidence: {confidence:.2f}\n"
        )

    print(string_to_print)


def main():
    print("Starting robot detection application...")

    user_data = user_app_callback_class()

    app = GStreamerDetectionApp(
        app_callback,
        user_data,
    )

    app.run()


if __name__ == "__main__":
    main()
