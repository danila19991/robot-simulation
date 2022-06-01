

class EventList:

    def __init__(self):
        self.event_list = list()
        self.event_dict = dict()

    def add_event(self, name):
        self.event_dict[name] = len(self.event_list)
        self.event_list.append(name)
