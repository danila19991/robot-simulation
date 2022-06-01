

class DataSingleton:
    __ins = None

    def __init__(self):
        if self.__ins is not None:
            raise AttributeError
        self.data = dict()

    @classmethod
    def get(cls):
        if not cls.__ins:
            cls.__ins = DataSingleton()
        return cls.__ins

    def get_ds(self, name):
        if name not in self.data:
            self.data[name] = list()
        return self.data[name]
