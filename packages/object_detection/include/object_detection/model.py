import numpy as np
import cv2

class Model():
    def __init__(self):
        pass
    def infer(self, image):
        raise NotImplementedError()

class AMD64Model():
    def __init__(self, weight_file_path):
        raise NotImplementedError()
    def infer(self, image):
        raise NotImplementedError()

class TRTModel(Model):
    def __init__(self, weight_file_path):
        raise NotImplementedError()
    def infer(self, image):
        raise NotImplementedError()



