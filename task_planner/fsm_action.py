

class BaseAction:

    def __init__(self):
        self.next_state = None
        self.start_time = None
        self.iterations = 1000

    def is_finished(self, t):
        return t > self.start_time + self.iterations

    @classmethod
    def is_fsm_state(cls):
        return False


class BaseActionGenerator:

    def __init__(self, curr, nxt):
        self.curr = curr
        self.next_state = nxt
        self.expect_time = 1000

    def generate_controller(self, start_time):
        raise NotImplementedError

    @classmethod
    def is_fsm_state(cls):
        return False

