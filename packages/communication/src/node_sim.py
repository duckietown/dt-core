import cv2
from BaseComNode import BaseComNode
from io import BytesIO
import time

#TEST_VIDEO = '/home/sh3mm/Desktop/IFT6757/image/videos/video_1.avi'
TEST_VIDEO = '/home/alaeddine/1IFT6757/Project/VideoCaptures/video-TL.avi'


class Data:
    def __init__(self, data):
        self.data = data


def main():
    vid = cv2.VideoCapture(TEST_VIDEO)

    node = BaseComNode()
    node.intersection_type_callback(2)
    counter = 0
    fps = 30
    while vid.isOpened():
        ret, curr_img = vid.read()
        if not ret:
            break

        # data prep to simulate the actual input
        _, buffer = cv2.imencode(".jpeg", curr_img)
        io_buf = BytesIO(buffer)
        data = Data(io_buf.getbuffer())

        # node call
        node.img_callback(data)

        node.run()

        time.sleep(1/fps) # Simulate 30 FPS

        if counter % fps == 0:
            print(f"Time: {counter/fps}")
        counter += 1

if __name__ == '__main__':
    main()
