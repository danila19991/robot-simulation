from robots.two_wheel.fsm_states import TwoWheelState
from robots.two_wheel.fsm_actions import CircleMoveGenerator, LinMoveGenerator
from task_planner.fsm_action import BaseActionGenerator
from task_planner.fsm_state import BaseFsmState


class CartMove:

    def __init__(self, start, end, next_choose):
        self.t1 = [1000]
        self.t2 = [0]
        self.next_choose = next_choose

        # todo fill params
        # self.sub_state = [TwoWheelState(None, None, None, None), TwoWheelState(None, None, None, None)]
        # self.act = [CircleMoveGenerator(None, start, self.sub_state[0], None),
        #             LinMoveGenerator(self.sub_state[0], self.sub_state[1], None),
        #             CircleMoveGenerator(None, self.sub_state[1], end, None)]
        # self.sub_state[0].default = self.act[1]
        # self.sub_state[1].default = self.act[2]
        self.act = [BaseActionGenerator(start, end)]

    def first_action(self):
        return self.act[0]


class CubeTransition1:

    def __init__(self, start_cart, end_cart, start_manip, end_manip, next_choose, manip_id):
        self.t1 = [0, 800]
        self.t2 = [1000, 1000]
        self.manip_id = manip_id
        self.next_choose = next_choose

        # todo actions
        self.act = [BaseActionGenerator(start_cart, end_cart),
                    BaseActionGenerator(start_manip, end_manip)]

    def first_action_c(self):
        return self.act[0]

    def first_action_m(self):
        return self.act[1]


class CubeTransition2:

    def __init__(self, start_cart, end_cart, start_manip, end_manip, next_choose, manip_id):
        self.t1 = [500, 500]
        self.t2 = [1000, 1000]
        self.manip_id = manip_id
        self.next_choose = next_choose

        # todo actions
        self.act = [BaseActionGenerator(start_cart, end_cart),
                    BaseActionGenerator(start_manip, end_manip)]

    def first_action_c(self):
        return self.act[0]

    def first_action_m(self):
        return self.act[1]
