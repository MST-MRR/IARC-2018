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
# number of image pyramid levels to use for detection
NUM_PYRAMID_LEVELS = 1

def main():
    '''Plays a video clip from Jimmy Fallon until the end of the video is reached, CTRL+C is pressed, or the user
      hits the designated quit key'''

    with mrrdt_vision.VideoReader(JIMMY_FALLON_VIDEO) as video, mrrdt_vision.Window(JIMMY_FALLON_VIDEO) as window:
        try:
            should_quit = False

            for frame in video:
                if should_quit:
                    break
                
                frame = cv2.resize(frame, RESIZE_TO)
                detections = mrrdt_vision.detect_object(frame, mrrdt_vision.ObjectTypes.FACE, num_pyramid_levels=NUM_PYRAMID_LEVELS)
                mrrdt_vision.draw_bounding_boxes(frame, detections)

                window.show(frame)

                should_quit = window.is_key_down(QUIT_KEY)
        except KeyboardInterrupt as e:
            pass


if __name__ == '__main__':
    main()
