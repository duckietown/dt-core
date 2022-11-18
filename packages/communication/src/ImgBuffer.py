from typing import List, Set
import numpy as np
from img_analysis import get_maxes, group_point, Point


class ImgBuffer:
    def __init__(self, buffer_size: int):
        self.buffer_size = buffer_size

        self.raw_imgs: List[np.ndarray] = []
        self.diff_imgs: List[np.ndarray] = []
        self.points: Set[Point] = set()

    def push(self, new_img: np.ndarray):
        if len(self.raw_imgs) > 0:
            diff = np.abs(new_img.astype(int) - self.raw_imgs[-1])
            diff[diff < 50] = 0
            self.diff_imgs.append(diff)

        if len(self.diff_imgs) > 0:
            max_points = get_maxes(self.diff_imgs[-1])
            self.points = set(group_point([*max_points, *self.points], 6))
        
        for point in self.points:
            point.last_seen += 1

        self.raw_imgs.append(new_img)
        
        self._trim_size()

    def _trim_size(self):
        if len(self.raw_imgs) > self.buffer_size:
            self.raw_imgs.pop(0)
        if len(self.diff_imgs) > self.buffer_size:
            self.diff_imgs.pop(0)

        for point in self.points.copy():
            if point.last_seen > self.buffer_size:
                self.points.remove(point)
