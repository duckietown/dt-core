import numpy as np
import cv2

from ImgBuffer import ImgBuffer

TEST_VIDEO = '/home/sh3mm/Desktop/IFT6757/image/videos/video_1.avi'


class NodeSim:
    def __init__(self):
        self.buffer = ImgBuffer(60, 40)

    def callback_sim(self, new_img: np.ndarray):
        self.buffer.push(new_img)

        for i, point in enumerate(self.buffer.points):
            print(f"{i} -- freq: {point.get_frequency()[0]} -- {point}")


def main():
    vid = cv2.VideoCapture('/home/sh3mm/Desktop/IFT6757/image/videos/video_1.avi')

    node = NodeSim()
    while vid.isOpened():
        ret, curr_img = vid.read()
        if not ret:
            break

        node.callback_sim(cv2.cvtColor(curr_img, cv2.COLOR_BGR2GRAY))


if __name__ == '__main__':
    main()
