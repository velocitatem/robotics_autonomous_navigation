#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def nothing(x):
    pass


class HSVCalibrator:
    def __init__(self):
        rospy.init_node("hsv_calibrator", anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_callback
        )
        self.latest_image = None
        self.crop_top_fraction = float(rospy.get_param("~crop_top_fraction", 0.20))

        cv2.namedWindow("HSV Calibration")
        cv2.createTrackbar("HMin", "HSV Calibration", 0, 179, nothing)
        cv2.createTrackbar("SMin", "HSV Calibration", 0, 255, nothing)
        cv2.createTrackbar("VMin", "HSV Calibration", 0, 255, nothing)
        cv2.createTrackbar("HMax", "HSV Calibration", 179, 179, nothing)
        cv2.createTrackbar("SMax", "HSV Calibration", 255, 255, nothing)
        cv2.createTrackbar("VMax", "HSV Calibration", 255, 255, nothing)

        rospy.loginfo("HSV Calibrator Started. Adjust trackbars to find target ranges.")

    def image_callback(self, data):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            print(e)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.latest_image is not None:
                frame = self.latest_image.copy()
                h = frame.shape[0]
                crop_top = min(h - 1, int(round(h * self.crop_top_fraction)))
                if crop_top > 0:
                    frame = frame[crop_top:, :]
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                h_min = cv2.getTrackbarPos("HMin", "HSV Calibration")
                s_min = cv2.getTrackbarPos("SMin", "HSV Calibration")
                v_min = cv2.getTrackbarPos("VMin", "HSV Calibration")
                h_max = cv2.getTrackbarPos("HMax", "HSV Calibration")
                s_max = cv2.getTrackbarPos("SMax", "HSV Calibration")
                v_max = cv2.getTrackbarPos("VMax", "HSV Calibration")

                lower = np.array([h_min, s_min, v_min])
                upper = np.array([h_max, s_max, v_max])

                mask = cv2.inRange(hsv, lower, upper)
                result = cv2.bitwise_and(frame, frame, mask=mask)

                # Show side by side
                stacked = np.hstack((frame, result))

                # Resize for easier viewing
                scale_percent = 50
                width = int(stacked.shape[1] * scale_percent / 100)
                height = int(stacked.shape[0] * scale_percent / 100)
                dim = (width, height)
                resized = cv2.resize(stacked, dim, interpolation=cv2.INTER_AREA)

                cv2.imshow("HSV Calibration", resized)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            rate.sleep()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        calibrator = HSVCalibrator()
        calibrator.run()
    except rospy.ROSInterruptException:
        pass
