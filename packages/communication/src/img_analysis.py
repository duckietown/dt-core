from typing import NamedTuple, Tuple, List, Iterable
from itertools import chain
import scipy.ndimage.filters as filters
import numpy as np
import cv2


class Point:
    def __init__(self, new_coords: Tuple[int, int], last_seen: int = 0, weight: int = 1):
        self.coords = new_coords
        self.last_seen = last_seen
        self.weight = weight

    def __add__(self, other: 'Point'):
        new_weight = other.weight + self.weight
        new_coords = (
            (other.coords[0] * other.weight + self.coords[0] * self.weight) // new_weight,
            (other.coords[1] * other.weight + self.coords[1] * self.weight) // new_weight
        )
        new_last_seen = min(other.last_seen, self.last_seen)

        return Point(new_coords, new_last_seen, new_weight)
    
    def __repr__(self):
        return f"coords: {self.coords}, last_seen: {self.last_seen}, weight: {self.weight}"

    @staticmethod
    def batch(batch: Iterable[Tuple[int, int]]) -> List['Point']:
        return [Point(b) for b in batch]


def group_point(points: List[Point], size):
    group_nb = 0
    groups = []
    for p1 in points:
        i1, j1 = p1.coords
        for p2, nb in chain(*groups):
            i2, j2 = p2.coords
            if abs(i2 - i1) < size and abs(j2 - j1) < size:
                groups[nb].append((p1, nb))
                break
        else:
            groups.append([(p1, group_nb)])
            group_nb += 1

    return [sum((e[0] for e in group[1:]), start=group[0][0]) for group in groups]


def get_maxes(base_img):
    size = 9
    threshold = 120

    gaussian_blur = cv2.GaussianBlur(src=np.uint8(base_img), ksize=(size, size), sigmaX=3, sigmaY=3)
    data_max = filters.maximum_filter(gaussian_blur, size)

    maxima = (gaussian_blur == data_max)

    diff = ((data_max - 0) > threshold)
    maxima[diff == 0] = 0

    return group_point(Point.batch(zip(*maxima.nonzero())), 2)


def get_sub(img_buffer: Iterable[np.ndarray], center: Point, radius: int) -> np.ndarray:
    return np.array(img_buffer)[
        :,
        center.coords[0] - radius: center.coords[0] + radius + 1,
        center.coords[1] - radius: center.coords[1] + radius + 1
    ]


def get_frequency(img_buffer: np.ndarray, frame_rate=30, tolerance=.30) -> Tuple[float, np.ndarray]:
    histogram = np.sum(img_buffer, axis=(1, 2), dtype=int)
    first_diff = np.clip(np.diff(histogram), 0, np.inf)
    second_diff = np.clip(np.diff(first_diff), 0, np.inf)

    if not np.sum(second_diff > 5e3):
        return -1, (second_diff > 5e3).astype(int)

    spikes = (second_diff / np.max(second_diff)) > tolerance
    spike_freq = frame_rate / np.median(np.diff(spikes.nonzero()))

    return spike_freq, spikes.astype(int)
