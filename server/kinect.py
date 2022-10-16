import freenect
import cv2
import io
import numpy as np


class Kinect:
    depth_meters = []
    kinect_max_range_meters = 6

    def generate_depth_meters_array(self):
        for i in range(0, 2048):
            mix = self.depth_to_meters(i) / self.kinect_max_range_meters
            if mix < 0:
                mix = self.kinect_max_range_meters

            self.depth_meters.append(
                self.color_fade(0, 255, mix=mix)
            )

    def get_image(self, depth=False):
        if depth:
            image, _t = freenect.sync_get_depth()
            image = self.pretty_depth(image)
        else:
            image, _t = freenect.sync_get_video()
            image = self.video_cv(image)

        return image

    def video_cv(self, video):
        return video[:, :, ::-1]  # RGB -> BGR

    def depth_to_meters(self, depth):
        return 1.0 / (depth * -0.0030711016 + 3.3309495161)

    def color_fade(self, c1, c2, mix=0.0):
        return(1-mix)*c1 + mix*c2

    def pretty_depth(self, depth):
        if not len(self.depth_meters):
            self.generate_depth_meters_array()
        for i, col in enumerate(depth):
            for j, value in enumerate(col):
                depth[i][j] = self.depth_meters[value]
        return depth

    def get_jpeg(self, depth=False) -> io.BytesIO:
        _is_success, buffer = cv2.imencode(".jpg", self.get_image(depth))
        io_buf = io.BytesIO(buffer)
        return io_buf

    def set_led(self, led):
        ctx = freenect.init()
        dev = freenect.open_device(ctx, 0)
        freenect.set_led(dev, led)
        freenect.close_device(dev)
