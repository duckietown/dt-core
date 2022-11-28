

class Model():
    def __init__(self):
        pass
    def infer(self, image):
        raise NotImplementedError()

class SimpleModel():
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



