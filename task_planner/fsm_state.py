
class GlobalIncrement:

    __instance = None

    def __init__(self):
        if self.__instance is not None:
            raise AttributeError
        self.i = 0

    @classmethod
    def get_instance(cls):
        if cls.__instance is None:
            cls.__instance = GlobalIncrement()
        return cls.__instance

    def get_id(self):
        self.i = self.i + 1
        return self.i - 1


class BaseFsmState:

    def __init__(self):
        self.next = dict()
        self.default = None
        self.id = GlobalIncrement.get_instance().get_id()
        self.name = f"state-{self.id}"

    def __str__(self):
        return self.name

    def check_event(self, e):
        return self.next.get(e, None)

    def use_default(self):
        return self.default

    @classmethod
    def is_finish(cls):
        return False

    @classmethod
    def is_start(cls):
        return False

    @classmethod
    def is_choose(cls):
        return False

    @classmethod
    def is_fsm_state(cls):
        return True


class ChooseMultiState(BaseFsmState):

    def __init__(self, states):
        super().__init__()
        self.name += '-choose'
        self.states = states

    @classmethod
    def is_choose(cls):
        return True

    def check_event(self, e):
        raise ValueError

    def use_default(self):
        raise ValueError


class StartState(BaseFsmState):

    def __init__(self):
        super().__init__()
        self.name += '-start'
        self.composite = None

    @classmethod
    def is_start(cls):
        return True


class EndState(BaseFsmState):

    def __init__(self):
        super().__init__()
        self.name += '-end'

    @classmethod
    def is_finish(cls):
        return True

