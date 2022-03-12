import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as b
from CameraCalibration.CalibrationConfig import *

AK = ArmIK()

range_rgb = {
    'red':   (0, 0, 255),
    'blue':  (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

def get_area_max_contour(contours):
        contour_area_temp = 0
        contour_area = 0
        area_contour = None

        for c in contours :
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area:
                contour_area = contour_area_temp
                if contour_area_temp > 300:
                    area_contour = c

        return area_contour, contour_area

class CubeTracker:
    def __init__(self, color: str = 'red'):
        self.target_color = color
        self.size = (640, 480)
        self.cube_pos = None
        self.camera = Camera.Camera()
        self.camera.camera_open()

    def track(self):
        frame = self.camera.frame
        img = frame.copy()
        img_h, img_w = img.shape[:2]

        # Blur, image format conversion stuff
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        # Find contours of pixels that match our target color range
        c_min, c_max = color_range[self.target_color]
        frame_mask = cv2.inRange(frame_lab, c_min, c_max)
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6,6),np.uint8))
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6,6),np.uint8))
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        area_contour, area = get_area_max_contour(contours)

        if area_contour is not None and area > 2500:
            # Get a bounding box for the contour
            rect = cv2.minAreaRect(area_contour)
            box = np.int0(cv2.boxPoints(rect))

            # Draw a bounding box around the contour and label it with the
            # converted coordinates
            roi = getROI(box)
            get_roi = True
            img_centerx, img_centery = getCenter(rect, roi, self.size, square_length)
            world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size)

            self.cube_position = world_x, world_y
            self.cube_rotation = rect[2]
            if draw:
                self.draw_track(img)
            return world_x, world_y, self.cube_rotation
        return None

    def draw_track(self, img):
        # Draw a line through the center of the screen for calibration (should align
        # with the blue cross on the paper)
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        world_x, world_y = self.cube_position
        cv2.drawContours(img, [box], -1, range_rgb[self.target_color], 2)
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[self.target_color], 1)

        cv2.putText(img, "Color: " + self.target_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, range_rgb[self.target_color], 2)

        cv2.imshow('Frame', img)
        key = cv2.waitKey(1)
        if key == 27:
            break

from enum import Enum

class State(Enum):
    WHERE_IS_BLOCK_I_DO_NOT_KNOW = 0
    FOUND_BLOCK_GETTING_READY = 1
    FOUND_BLOCK_MOVING_TO_POSITION = 2
    NEAR_BLOCK_MATCHING_ORIENTATION = 3
    NEAR_BLOCK_LOWERING= 4
    NEAR_BLOCK_GRABBING= 5
    HAVE_BLOCK_RAISING= 6
    HAVE_BLOCK_RETURNING_TO_DROPOFF= 7

def beep(num_beeps, time):
    b.setBuzzer(0)
    for i in range(2*num_beeps):
        if i % 2 == 0:
            b.setBuzzer(1)
        else:
            b.setBuzzer(0)
        time.sleep(time / (2 * num_beeps))
    b.setBuzzer(0)

class CubeGrabber:
    def __init__(self):
        # Possible states:

        self.state = State.WHERE_IS_BLOCK_I_DO_NOT_KNOW

        self.ik = ArmIK()
        beep(2, 1)
        self.dropoff = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }
        self.closed_gripper_angle = 500

    def grab(self, cube_pose):
        beep(self.state, 1)
        print('start state:', self.state)
        if cube_pose is not None:
            cube_position, cube_rotation = cube_pose[:2], cube_pose[2]

        if cube_pose is None:
            self.state = State.WHERE_IS_BLOCK_I_DO_NOT_KNOW

            # Initial position
            b.setBusServoPulse(1, servo1 - 50, 300)
            b.setBusServoPulse(2, 500, 500)
            self.ik.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

        elif self.state == State.WHERE_IS_BLOCK_I_DO_NOT_KNOW:
            # Open gripper
            self.state = State.FOUND_BLOCK_GETTING_READY
            b.setBusServoPulse(1, self.closed_gripper_angle - 280, 500)
            b.setBusServoPulse(2, servo2_angle, 500)

        elif self.state == State.FOUND_BLOCK_GETTING_READY:
            # Move arm to 7cm above surface over block
            if self.ik.setPitchRangeMoving((*cube_position, 7), -90, -90, 0):
                # Block is reachable
                self.state = State.FOUND_BLOCK_MOVING_TO_POSITION
            else:
                # Not reachable
                self.state = State.WHERE_IS_BLOCK_I_DO_NOT_KNOW

        elif self.state == State.FOUND_BLOCK_MOVING_TO_POSITION
            self.state = State.NEAR_BLOCK_MATCHING_ORIENTATION
            servo2_angle = getAngle(*cube_position, cube_rotation)
            b.setBusServoPulse(1, self.closed_gripper_angle - 280, 500)
            b.setBusServoPulse(2, servo2_angle, 500)
            time.sleep(0.5)

        elif self.state == State.NEAR_BLOCK_MATCHING_ORIENTATION:
            self.state = State.NEAR_BLOCK_LOWERING
            self.ik.setPitchRangeMoving((*cube_position, 1.5), -90, -90, 0, 1000)

        elif self.state = State.NEAR_BLOCK_LOWERING
            self.state = State.NEAR_BLOCK_GRABBING
            b.setBusServoPulse(1, 500, 500)

        elif self.state == State.NEAR_BLOCK_GRABBING:
            self.state = State.HAVE_BLOCK_RAISING
            self.ik.setPitchRangeMoving((*cube_position, 12), -90, -90, 0, 1000)

        elif self.state == State.HAVE_BLOCK_RAISING:
            self.state = State.HAVE_BLOCK_RETURNING_TO_DROPOFF

            # Move arm to just above dropoff location
            result = self.ik.setPitchRangeMoving((*self.dropoff['red'][:2], 12), -90, -90, 0)

            # Rotate gripper to match angle of dropoff location
            servo2_angle = getAngle(*self.dropoff['red'][:2], -90)
            b.setBusServoPulse(2, servo2_angle, 500)
            time.sleep(0.5)

            # Move gripper down to 3cm above target area
            self.ik.setPitchRangeMoving((*self.dropoff['red'] + [0, 0, 3]), -90, -90, 0, 500)
            time.sleep(0.5)

            # Move gripper down directly onto target area
            self.ik.setPitchRangeMoving((self.dropoff['red']), -90, -90, 0, 1000)
            time.sleep(0.8)

            # Open gripper to release cube
            b.setBusServoPulse(1,  self.closed_gripper_angle - 200, 500)
            time.sleep(0.8)

            # Lift cube back up
            self.ik.setPitchRangeMoving((*self.dropoff['red'][:2], 12), -90, -90, 0, 800)
            time.sleep(1.5)
        print('end state:', self.state)


if __name__ == '__main__':
    tracker = CubeTracker()
    grabber = CubeGrabber()

    threads = []
    timer = rossros.Timer((timer_bus,), duration=5, delay=0.1, termination_busses=(timer_bus,), name="master timer")
    tracking_bus = rossros.Bus(name='trackbus')
    threads += [rossros.Producer(tracker.track, tracking_bus, termination_busses=(timer_bus,), delay=dt, name='tracker')]
    threads += [rossros.Consumer(grabber.grab, tracking_bus, termination_busses=(timer_bus,), delay=dt, name='grabber')]
    rossros.runConcurrently(threads)

    tracker.camera.camera_close()
    cv2.destroyAllWindows()