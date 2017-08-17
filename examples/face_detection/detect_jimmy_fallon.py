############################################
# Multi-Rotor Robot Design Team
# Missouri University of Science and Technology
# Summer 2017
# Christopher O'Toole

'''Example showing how to use the mrrdt_vision module to detect faces in videos.'''

import sys
sys.path.append('../..')

import mrrdt_vision
import cv2

# path to Jimmy Fallon video
JIMMY_FALLON_VIDEO = 'jimmy.mp4'
# key to press to terminate the program
QUIT_KEY = 'q'
# resolution to resize frames from the video to
RESIZE_TO = (640, 480)
# bounding box color
GREEN = (0, 255, 0)
# bounding box thickness
THICKNESS = 3

class VideoReader():
    '''Utility class for reading a video file with OpenCV'''

    def __init__(self, file_name):
        '''Intialize internal attributes'''
        self.file_name = file_name

    def __enter__(self):
        '''Begin streaming from the video on disk once instantiated by a context manager'''
        self.cap = cv2.VideoCapture(self.file_name)
        return self

    def next(self):
        '''
        Gets the next frame from the video.

        Returns
        -------
        out: numpy.ndarray or None
        Returns the next frame, or None if the video is finished playing.

        Notes
        ------
        This function may also return None when the video file does not exist, so be sure to check
        that python can find the file by using os.path.isfile(path_to_file) if the video does not
        seem to be playing.
        '''

        if not self.cap.isOpened():
            return None

        return self.cap.read()[1]

    def __exit__(self, *args):
        '''Stop streaming from the video on disk once the context manager's scope ends or an exception is hit'''
        self.cap.release()

def main():
    '''Plays a video clip from Jimmy Fallon until the end of the video is reached, CTRL+C is pressed, or the user
      hits the designated quit key'''

    with VideoReader(JIMMY_FALLON_VIDEO) as video:
        try:
            frame = video.next()
            should_quit = False

            while frame is not None and not should_quit:
                frame = cv2.resize(frame, RESIZE_TO)
                detections = mrrdt_vision.detect_object(frame, mrrdt_vision.ObjectTypes.FACE, num_pyramid_levels=1)

                for (x_min, y_min, x_max, y_max) in detections:
                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), GREEN, THICKNESS)

                cv2.imshow(JIMMY_FALLON_VIDEO, frame)

                # the & 0xFF is necessary for getting the correct character on 64-bit machines
                should_quit = (chr(cv2.waitKey(1) & 0xFF) == QUIT_KEY)
                frame = video.next()
        except KeyboardInterrupt as e:
            pass
        finally:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
