import pyrealsense2 as rs

import cv2

import numpy as np



REALSENSE_STREAM_WINDOW = 'librealsense 2.0 viewer'



class Realsense():

    NUMBER_OF_TRIES = 5

    COLORMAP_ALPHA = .03

    FPS = 60



    def __init__(self, color_res, depth_res=None, fps=FPS, color_format=rs.format.bgr8, depth_format=rs.format.z16):

        if depth_res is None:

            depth_res = color_res



        print(color_res, depth_res, fps, color_format, depth_format)



        self._pipeline = None

        self.color_res = color_res

        self.depth_res = depth_res

        self.fps = fps

        self.color_format = color_format

        self.depth_format = depth_format



    @property

    def color_res(self):

        return self._color_res

    

    @property

    def depth_res(self):

        return self._depth_res



    @property

    def fps(self):

        return self._fps



    @property

    def color_format(self):

        return self._color_format



    @property

    def depth_format(self):

        return self._depth_format



    @color_res.setter

    def color_res(self, value):

        self._color_res = value

    

    @depth_res.setter

    def depth_res(self, value):

        self._depth_res = value

    

    @fps.setter

    def fps(self, value):

        self._fps = value



    @color_format.setter

    def color_format(self, value):

        self._color_format = value



    @depth_format.setter

    def depth_format(self, value):

        self._depth_format = value

    

    def __enter__(self):

        self._pipeline = rs.pipeline()

        config = rs.config()

        config.enable_stream(rs.stream.color, self.color_res[0], self.color_res[1], self.color_format, self.fps)

        config.enable_stream(rs.stream.depth, self.depth_res[0], self.depth_res[1], self.depth_format, self.fps)

        self._pipeline.start(config)

        return self



    def __exit__(self, *args):

        if self._pipeline is not None:

            self._pipeline.stop()





    def next(self, tries=NUMBER_OF_TRIES):

        frames = self._pipeline.wait_for_frames()

        depth = frames.get_depth_frame()

        color = frames.get_color_frame()



        if (not depth or not color) and tries > 0:

            return self.next()

        

        depth_image = np.asanyarray(depth.get_data())

        color_image = np.asanyarray(color.get_data())

        return depth and color, color_image, depth_image

    

    def render(self, color, depth=None, window_name=REALSENSE_STREAM_WINDOW, colormap_alpha=COLORMAP_ALPHA):

        images = color



        if depth is not None:

            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=colormap_alpha), cv2.COLORMAP_JET)

            images = np.hstack((images, depth_colormap))

        

        cv2.imshow(window_name, images)

        cv2.waitKey(1)



if __name__ == '__main__':

    with Realsense((640, 480)) as realsense:

        try:

            cv2.namedWindow(REALSENSE_STREAM_WINDOW, cv2.WINDOW_AUTOSIZE)



            while True:

                status, color, depth = realsense.next()

                if status:

                    realsense.render(color, depth)

        except KeyboardInterrupt as e:

            print('Quitting...')
